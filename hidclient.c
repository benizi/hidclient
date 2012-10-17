/*
 * hidclient - A Bluetooth HID client device emulation
 *
 *		A bluetooth/bluez software emulating a bluetooth mouse/keyboard
 *		combination device - use it to transmit mouse movements and
 *		keyboard keystrokes from a Linux box to any other BTHID enabled
 *		computer or compatible machine
 * 
 * 
 * Implementation
 * 2004-2006	by Anselm Martin Hoffmeister
 * 		   <stockholm(at)users(period)sourceforge(period)net>
 * 2004/2005	Implementation for the GPLed Nokia-supported Affix Bluetooth
 * 		stack as a practical work at
 * 		Rheinische Friedrich-Wilhelms-UniversitÃ€t, Bonn, Germany
 * 2006		Software ported to Bluez, several code cleanups
 * 2006-07-25	First public release
 * 
 * Updates
 * 2012-02-10	by Peter G
 *		Updated to work with current distro (Lubuntu 11.10)
 *		EDIT FILE /etc/bluetooth/main.conf
 *		to include:	
 *			DisablePlugins = network,input,hal,pnat
 *		AMH: Just disable input might be good enough.
 *		recomended to change:
 *			Class = 0x000540
 *		AMH: This leads to the device being found as "Keyboard".
 *		Some devices might ignore non-0x540 input devices
 *		before starting, also execute the following commands
 *			hciconfig hci0 class 0x000540
 *			# AMH: Should be optional if Class= set (above)
 *			hciconfig hci0 name \'Bluetooth Keyboard\'
 *			# AMH: Optional: Name will be seen by other BTs
 *			hciconfig hci0 sspmode 0
 *			# AMH: Optional: Disables simple pairing mode
 *			sdptool del 0x10000
 *			sdptool del 0x10001
 *			sdptool del 0x10002
 *			sdptool del 0x10003
 *			sdptool del 0x10004
 *			sdptool del 0x10005
 *			sdptool del 0x10006
 *			sdptool del 0x10007
 *			sdptool del 0x10008
 *			# This removes any non-HID SDP entries.
 *			# Might help if other devices only like
 *			# single-function Bluetooth devices
 *			# Not strictly required though.
 * 2012-07-26	Several small updates necessary to work on
 *		Ubuntu 12.04 LTS on amd64
 *		Added -e, -l, -x
 * 2012-07-28	Add support for FIFO operation (-f/filename)
 * 
 * Dependency:	Needs libbluetooth (from bluez)
 *
 * Usage:	hidclient [-h|-?|--help] [-s|--skipsdp]
 * 		Start hidclient. -h will display usage information.
 *		-e<NUM> asks hidclient to ONLY use Input device #NUM
 *		-f<FILENAME> will not read event devices, but create a
 *		   fifo on <FILENAME> and read input_event data blocks
 *		   from there
 *		-l will list input devices available
 *		-x will try to remove the "grabbed" input devices from
 *		   the local X11 server, if possible
 * 		-s will disable SDP registration (which only makes sense
 * 		when debugging as most counterparts require SDP to work)
 * Tip:		Use "openvt" along with hidclient so that keystrokes and
 * 		mouse events captured will have no negative impact on the
 * 		local machine (except Ctrl+Alt+[Fn/Entf/Pause]).
 * 		Run
 * 		openvt -s -w hidclient
 * 		to run hidclient on a virtual text mode console for itself.
 * Alternative:	Use "hidclient" in a X11 session after giving the active
 *		user read permissions on the /dev/input/event<NUM> devices
 *		you intend to use. With the help of -x (and possibly -e<NUM>)
 *		this should simply disattach input devices from the local
 *		machine while hidclient is running (and hopefully, reattach
 *		them afterwards, of course).
 * 		
 * License:
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License as
 *		published by the Free Software Foundation;
 *		strictly version 2 only.
 *
 *		This program is distributed in the hope that it will be useful,
 *		but WITHOUT ANY WARRANTY; without even the implied warranty of
 *		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *		GNU General Public License for more details.
 *
 *		You should have received a copy of the GNU General Public
 *		License	along with this program; if not, write to the
 *		Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 *		Boston, MA  02110-1301  USA
 */
//***************** Include files
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stropts.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <linux/input.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/sdp.h>
#include <bluetooth/sdp_lib.h>

//***************** Static definitions
// Where to find event devices (that must be readable by current user)
// "%d" to be filled in, opening several devices, see below
#define	EVDEVNAME	"/dev/input/event%d"

// Maximally, read MAXEVDEVS event devices simultaneously
#define	MAXEVDEVS 16

// Bluetooth "ports" (PSMs) for HID usage, standardized to be 17 and 19 resp.
// In theory you could use different ports, but several implementations seem
// to ignore the port info in the SDP records and always use 17 and 19. YMMV.
#define	PSMHIDCTL	0x11
#define	PSMHIDINT	0x13

// Information to be submitted to the SDP server, as service description
#define	HIDINFO_NAME	"Bluez virtual Mouse and Keyboard"
#define	HIDINFO_PROV	"Anselm Martin Hoffmeister (GPL v2)"
#define	HIDINFO_DESC	"Keyboard"

// These numbers must also be used in the HID descriptor binary file
#define	REPORTID_MOUSE	1
#define	REPORTID_KEYBD	2

// Fixed SDP record, corresponding to data structures below. Explanation
// is in separate text file. No reason to change this if you do not want
// to fiddle with the data sent over the BT connection as well.
#define SDPRECORD	"\x05\x01\x09\x02\xA1\x01\x85\x01\x09\x01\xA1\x00" \
			"\x05\x09\x19\x01\x29\x03\x15\x00\x25\x01\x75\x01" \
			"\x95\x03\x81\x02\x75\x05\x95\x01\x81\x01\x05\x01" \
			"\x09\x30\x09\x31\x09\x38\x15\x81\x25\x7F\x75\x08" \
			"\x95\x02\x81\x06\xC0\xC0\x05\x01\x09\x06\xA1\x01" \
			"\x85\x02\xA1\x00\x05\x07\x19\xE0\x29\xE7\x15\x00" \
			"\x25\x01\x75\x01\x95\x08\x81\x02\x95\x08\x75\x08" \
			"\x15\x00\x25\x65\x05\x07\x19\x00\x29\x65\x81\x00" \
			"\xC0\xC0"
#define SDPRECORD_BYTES	98

//***************** Function prototypes
int		dosdpregistration(void);
void		sdpunregister(unsigned int);
static void	add_lang_attr(sdp_record_t *r);
int		btbind(int sockfd, unsigned short port);
int		initevents(int,int);
void		closeevents(void);
int		initfifo(char *);
void		closefifo(void);
void		cleanup_stdin(void);
int		add_filedescriptors(fd_set*);
int		parse_events(fd_set*,int);
void		showhelp(void);
void		onsignal(int);

//***************** Data structures
// Mouse HID report, as sent over the wire:
struct hidrep_mouse_t
{
	unsigned char	btcode;	// Fixed value for "Data Frame": 0xA1
	unsigned char	rep_id; // Will be set to REPORTID_MOUSE for "mouse"
	unsigned char	button;	// bits 0..2 for left,right,middle, others 0
	signed   char	axis_x; // relative movement in pixels, left/right
	signed   char	axis_y; // dito, up/down
	signed   char	axis_z; // Used for the scroll wheel (?)
} __attribute((packed));
// Keyboard HID report, as sent over the wire:
struct hidrep_keyb_t
{
	unsigned char	btcode; // Fixed value for "Data Frame": 0xA1
	unsigned char	rep_id; // Will be set to REPORTID_KEYBD for "keyboard"
	unsigned char	modify; // Modifier keys (shift, alt, the like)
	unsigned char	key[8]; // Currently pressed keys, max 8 at once
} __attribute((packed));

//***************** Global variables
char		prepareshutdown	 = 0;	// Set if shutdown was requested
int		eventdevs[MAXEVDEVS];	// file descriptors
int		x11handles[MAXEVDEVS];
char		mousebuttons	 = 0;	// storage for button status
char		modifierkeys	 = 0;	// and for shift/ctrl/alt... status
char		pressedkey[8]	 = { 0, 0, 0, 0,  0, 0, 0, 0 };
char		connectionok	 = 0;
uint32_t	sdphandle	 = 0;	// To be used to "unregister" on exit
int		debugevents      = 0;	// bitmask for debugging event data

//***************** Implementation
/* 
 * Taken from bluetooth library because of suspicious memory allocation
 * THIS IS A HACK that appears to work, and did not need to look into how it works
 * SHOULD BE FIXED AND FIX BACKPORTED TO BLUEZ
 */
sdp_data_t *sdp_seq_alloc_with_length(void **dtds, void **values, int *length,
								int len)
{
	sdp_data_t *curr = NULL, *seq = NULL;
	int i;
	int totall = 1024;

	for (i = 0; i < len; i++) {
		sdp_data_t *data;
		int8_t dtd = *(uint8_t *) dtds[i];


		if (dtd >= SDP_SEQ8 && dtd <= SDP_ALT32) {
			data = (sdp_data_t *) values[i]; }
		else {
			data = sdp_data_alloc_with_length(dtd, values[i], length[i]); }

		if (!data)
			return NULL;

		if (curr)
			curr->next = data;
		else
			seq = data;

		curr = data;
		totall +=  length[i] + sizeof *seq; /* no idea what this should really be */
	}
/*
 * Here we had a reference here to a non-existing array member. Changed it something that
 * appears to be large enough BUT author has no idea what it really should be
 */
//  fprintf ( stderr, "length[%d]): %d, totall: %d\n", i, length[i], totall);

	return sdp_data_alloc_with_length(SDP_SEQ8, seq, totall);
}

/*
 * dosdpregistration -	Care for the proper SDP record sent to the "sdpd"
 *			so that other BT devices can discover the HID service
 * Parameters: none; Return value: 0 = OK, >0 = failure
 */
int	dosdpregistration ( void )
{
	sdp_record_t	record;
	sdp_session_t	*session;
        sdp_list_t	*svclass_id,
			*pfseq,
			*apseq,
			*root;
	uuid_t		root_uuid,
			hidkb_uuid,
			l2cap_uuid,
			hidp_uuid;
        sdp_profile_desc_t	profile[1];
        sdp_list_t	*aproto,
			*proto[3];
	sdp_data_t	*psm,
			*lang_lst,
			*lang_lst2,
			*hid_spec_lst,
			*hid_spec_lst2;
        void		*dtds[2],
			*values[2],
			*dtds2[2],
			*values2[2];
        int		i,
			leng[2];
        uint8_t		dtd=SDP_UINT16,
			dtd2=SDP_UINT8,
			dtd_data=SDP_TEXT_STR8,
			hid_spec_type=0x22;
        uint16_t	hid_attr_lang[]={0x409, 0x100},
			ctrl=PSMHIDCTL,
			intr=PSMHIDINT,
			hid_attr[]={0x100, 0x111, 0x40, 0x00, 0x01, 0x01},
			// Assigned to SDP 0x200...0x205 - see HID SPEC for
			// details. Those values seem to work fine...
			// "it\'s a kind of magic" numbers.
			hid_attr2[]={0x100, 0x0};

	// Connect to SDP server on localhost, to publish service information
	session = sdp_connect ( BDADDR_ANY, BDADDR_LOCAL, 0 );
	if ( ! session )
	{
		fprintf ( stderr, "Failed to connect to SDP server: %s\n",
				strerror ( errno ) );
		return	1;
	}
        memset(&record, 0, sizeof(sdp_record_t));
        record.handle = 0xffffffff;
	// With 0xffffffff, we get assigned the first free record >= 0x10000
	// Make HID service visible (add to PUBLIC BROWSE GROUP)
        sdp_uuid16_create(&root_uuid, PUBLIC_BROWSE_GROUP);
        root = sdp_list_append(0, &root_uuid);
        sdp_set_browse_groups(&record, root);
	// Language Information to be added
        add_lang_attr(&record);
	// The descriptor for the keyboard
        sdp_uuid16_create(&hidkb_uuid, HID_SVCLASS_ID);
        svclass_id = sdp_list_append(0, &hidkb_uuid);
        sdp_set_service_classes(&record, svclass_id);
	// And information about the HID profile used
        sdp_uuid16_create(&profile[0].uuid, HIDP_UUID /*HID_PROFILE_ID*/);
        profile[0].version = 0x0100;
        pfseq = sdp_list_append(0, profile);
        sdp_set_profile_descs(&record, pfseq);
	// We are using L2CAP, so add an info about that
        sdp_uuid16_create(&l2cap_uuid, L2CAP_UUID);
        proto[1] = sdp_list_append(0, &l2cap_uuid);
        psm = sdp_data_alloc(SDP_UINT16, &ctrl);
        proto[1] = sdp_list_append(proto[1], psm);
        apseq = sdp_list_append(0, proto[1]);
	// And about our purpose, the HID protocol data transfer
        sdp_uuid16_create(&hidp_uuid, HIDP_UUID);
        proto[2] = sdp_list_append(0, &hidp_uuid);
        apseq = sdp_list_append(apseq, proto[2]);
        aproto = sdp_list_append(0, apseq);
        sdp_set_access_protos(&record, aproto);
        proto[1] = sdp_list_append(0, &l2cap_uuid);
        psm = sdp_data_alloc(SDP_UINT16, &intr);
        proto[1] = sdp_list_append(proto[1], psm);
        apseq = sdp_list_append(0, proto[1]);
        sdp_uuid16_create(&hidp_uuid, HIDP_UUID);
        proto[2] = sdp_list_append(0, &hidp_uuid);
        apseq = sdp_list_append(apseq, proto[2]);
        aproto = sdp_list_append(0, apseq);
        sdp_set_add_access_protos(&record, aproto);
	// Set service name, description
        sdp_set_info_attr(&record, HIDINFO_NAME, HIDINFO_PROV, HIDINFO_DESC);
	// Add a few HID-specifid pieces of information
        // See the HID spec for details what those codes 0x200+something
	// are good for... we send a fixed set of info that seems to work
        sdp_attr_add_new(&record, SDP_ATTR_HID_DEVICE_RELEASE_NUMBER,
                                        SDP_UINT16, &hid_attr[0]); /* Opt */
        sdp_attr_add_new(&record, SDP_ATTR_HID_PARSER_VERSION,
                                        SDP_UINT16, &hid_attr[1]); /* Mand */
        sdp_attr_add_new(&record, SDP_ATTR_HID_DEVICE_SUBCLASS,
                                        SDP_UINT8, &hid_attr[2]); /* Mand */
        sdp_attr_add_new(&record, SDP_ATTR_HID_COUNTRY_CODE,
                                        SDP_UINT8, &hid_attr[3]); /* Mand */
        sdp_attr_add_new(&record, SDP_ATTR_HID_VIRTUAL_CABLE,
                                  SDP_BOOL, &hid_attr[4]); /* Mand */
        sdp_attr_add_new(&record, SDP_ATTR_HID_RECONNECT_INITIATE,
                                  SDP_BOOL, &hid_attr[5]); /* Mand */
	// Add the HID descriptor (describing the virtual device) as code
	// SDP_ATTR_HID_DESCRIPTOR_LIST (0x206 IIRC)
        dtds[0] = &dtd2;
        values[0] = &hid_spec_type;
	dtd_data= SDPRECORD_BYTES <= 255 ? SDP_TEXT_STR8 : SDP_TEXT_STR16 ;
        dtds[1] = &dtd_data;
        values[1] = (uint8_t *) SDPRECORD;
        leng[0] = 0;
        leng[1] = SDPRECORD_BYTES;
        hid_spec_lst = sdp_seq_alloc_with_length(dtds, values, leng, 2);
        hid_spec_lst2 = sdp_data_alloc(SDP_SEQ8, hid_spec_lst);
        sdp_attr_add(&record, SDP_ATTR_HID_DESCRIPTOR_LIST, hid_spec_lst2);
	// and continue adding further data bytes for 0x206+x values
        for (i = 0; i < sizeof(hid_attr_lang) / 2; i++) {
                dtds2[i] = &dtd;
                values2[i] = &hid_attr_lang[i];
        }
        lang_lst = sdp_seq_alloc(dtds2, values2, sizeof(hid_attr_lang) / 2);
        lang_lst2 = sdp_data_alloc(SDP_SEQ8, lang_lst);
        sdp_attr_add(&record, SDP_ATTR_HID_LANG_ID_BASE_LIST, lang_lst2);
	sdp_attr_add_new ( &record, SDP_ATTR_HID_PROFILE_VERSION,
			SDP_UINT16, &hid_attr2[0] );
	sdp_attr_add_new ( &record, SDP_ATTR_HID_BOOT_DEVICE,
			SDP_UINT16, &hid_attr2[1] );
	// Submit our IDEA of a SDP record to the "sdpd"
        if (sdp_record_register(session, &record, SDP_RECORD_PERSIST) < 0) {
                fprintf ( stderr, "Service Record registration failed\n" );
                return -1;
        }
	// Store the service handle retrieved from there for reference (i.e.,
	// deleting the service info when this program terminates)
	sdphandle = record.handle;
        fprintf ( stdout, "HID keyboard/mouse service registered\n" );
        return 0;
}

/*
 * 	sdpunregister - Remove SDP entry for HID service on program termination
 * 	Parameters: SDP handle (typically 0x10004 or similar)
 */
void	sdpunregister ( uint32_t handle )
{
        uint32_t	range=0x0000ffff;
	sdp_list_t    *	attr;
	sdp_session_t *	sess;
	sdp_record_t  *	rec;
	// Connect to the local SDP server
	sess = sdp_connect(BDADDR_ANY, BDADDR_LOCAL, 0);
	if ( !sess )	return;
	attr = sdp_list_append(0, &range);
	rec = sdp_service_attr_req(sess, handle, SDP_ATTR_REQ_RANGE, attr);
	sdp_list_free(attr, 0);
	if ( !rec ) {
		sdp_close(sess);
		return;
	}
	sdp_device_record_unregister(sess, BDADDR_ANY, rec);
	sdp_close(sess);
	// We do not care wether unregister fails. If it does, we cannot help it.
	return;
}

static void add_lang_attr(sdp_record_t *r)
{
        sdp_lang_attr_t base_lang;
        sdp_list_t *langs = 0;
        /* UTF-8 MIBenum (http://www.iana.org/assignments/character-sets) */
        base_lang.code_ISO639 = (0x65 << 8) | 0x6e;
        base_lang.encoding = 106;
        base_lang.base_offset = SDP_PRIMARY_LANG_BASE;
        langs = sdp_list_append(0, &base_lang);
        sdp_set_lang_attr(r, langs);
        sdp_list_free(langs, 0);
}

// Wrapper for bind, caring for all the surrounding variables
int	btbind ( int sockfd, unsigned short port ) {
	struct sockaddr_l2 l2a;
	int i;
	memset ( &l2a, 0, sizeof(l2a) );
	l2a.l2_family = AF_BLUETOOTH;
	bacpy ( &l2a.l2_bdaddr, BDADDR_ANY );
	l2a.l2_psm = htobs ( port );
	i = bind ( sockfd, (struct sockaddr *)&l2a, sizeof(l2a) );
	if ( 0 > i )
	{
		fprintf ( stderr, "Bind error (PSM %d): %s\n",
				port, strerror ( errno ) );
	}
	return	i;
}


/*
 *	initfifo(filename) - creates (if necessary) and opens fifo
 *	instead of event devices. If filename exists and is NOT a fifo,
 *	abort with error.
 */
int	initfifo ( char *filename )
{
	struct stat ss;
	if ( NULL == filename ) return 0;
	if ( 0 == stat ( filename, &ss ) )
	{
		if ( ! S_ISFIFO(ss.st_mode) )
		{
			fprintf(stderr,"File [%s] exists, but is not a fifo.\n", filename );
			return 0;
		}
	} else {
		if ( 0 != mkfifo ( filename, S_IRUSR | S_IWUSR ) )
		{	// default permissions for created fifo is rw------- (user=rw)
			fprintf(stderr,"Failed to create new fifo [%s]\n", filename );
			return 0;
		}
	}
	eventdevs[0] = open ( filename, O_RDONLY | O_NONBLOCK );
	if ( 0 > eventdevs[0] )
	{
		fprintf ( stderr, "Failed to open fifo [%s] for reading.\n", filename );
		return 0;
	}
	return	1;
}

/*
 * 	initevents () - opens all required event files
 * 	or only one device, if number useonlyone is >= 0
 *	try to disable in X11 if mutex11 is set to 1
 * 	returns number of successfully opened event file nodes, or <1 for error
 */
int	initevents ( int useonlyone, int mutex11 )
{
	int	i, j, k;
	char	buf[sizeof(EVDEVNAME)+8];
	char	*xinlist = NULL;
	FILE	*pf;
	char	*p, *q;
	if ( mutex11 )
	{
		if ( NULL == ( xinlist = malloc ( 4096 ) ) )
		{
			printf ( "Memory alloc error\n" );
			return 0;
		}
		bzero ( xinlist, 4096 );
		if ( NULL != ( pf = popen ("xinput --list --short", "r" ) ) )
		{
			if ( 1 > fread ( xinlist, 1, 3800, pf ) )
			{
				printf ( "\tx11-mutable information not available.\n" );
				free ( xinlist );
				xinlist = NULL;
			}
		}
		fclose ( pf );
	}
	for ( i = 0; i < MAXEVDEVS; ++i )
	{
		eventdevs[i] = -1;
		x11handles[i] = -1;
	}
	for ( i = j = 0; j < MAXEVDEVS; ++j )
	{
		if ( ( useonlyone >= 0 ) && ( useonlyone != j ) ) { continue; }
		sprintf ( buf, EVDEVNAME, j );
		eventdevs[i] = open ( buf, O_RDONLY );
		if ( 0 <= eventdevs[i] )
		{
			fprintf ( stdout, "Opened %s as event device [counter %d]\n", buf, i );
			if ( ( mutex11 > 0 ) && ( xinlist != NULL ) )
			{
				k = -1;
				xinlist[3801] = 0;
				if ( ioctl(eventdevs[i], EVIOCGNAME(256),xinlist+3801) >= 0 )
				{
					p = xinlist;
					xinlist[4056] = 0;
					if ( strlen(xinlist+3801) < 4 ) // min lenght for name
						p = xinlist + 4056;
					while ( (*p != 0) &&
						( NULL != ( p = strstr ( p, xinlist+3801 ) ) ) )
					{
						q = p + strlen(xinlist+3801);
						while ( *q == ' ' ) ++q;
						if ( strncmp ( q, "\tid=", 4 ) == 0 )
						{
							k = atoi ( q + 4 );
							p = xinlist + 4056;
						} else {
							p = q;
						}
					}
				}
				if ( k >= 0 ) {
					sprintf ( xinlist+3801, "xinput set-int-prop %d \"Device "\
						"Enabled\" 8 0", k );
					if ( system ( xinlist + 3801 ) )
					{
						fprintf ( stderr, "Failed to x11-mute.\n" );
					}
					x11handles[i] = k;
				}
			}
			++i;
		}
	}
	if ( xinlist != NULL ) { free ( xinlist ); }
	return	i;
}

void	closeevents ( void )
{
	int	i;
	char	buf[256];
	for ( i = 0; i < MAXEVDEVS; ++i )
	{
		if ( eventdevs[i] >= 0 )
		{
			close ( eventdevs[i] );
			if ( x11handles[i] >= 0 )
			{
				sprintf ( buf, "xinput set-int-prop %d \"Device "\
				"Enabled\" 8 1", x11handles[i] );
				if ( system ( buf ) )
				{
					fprintf ( stderr, "Failed to x11-unmute device %d.\n", i );
				}
			}
		}
	}
	return;
}

void	closefifo ( void )
{
	if ( eventdevs[0] >= 0 )
		close(eventdevs[0]);
	return;
}

void	cleanup_stdin ( void )
{
	// Cleans everything but the characters after the last ENTER keypress.
	// This is necessary BECAUSE while reading all keyboard input from the
	// event devices, those key presses will still stay in the stdin queue.
	// We do not want to have a backlog of hundreds of characters, possibly
	// commands and so on.
	fd_set fds;
	struct timeval tv;
	FD_ZERO ( &fds );
	FD_SET ( 0, &fds );
	tv.tv_sec  = 0;
	tv.tv_usec = 0;
	char buf[8];
	while ( 0 < select ( 1, &fds, NULL, NULL, &tv ) )
	{
		while ( read ( 0, buf, 8 ) ) {;}
		FD_ZERO ( &fds );
		FD_SET ( 0, &fds );
		tv.tv_sec  = 0;
		tv.tv_usec = 1;
	}
	close ( 0 );
	return;
}

int	add_filedescriptors ( fd_set * fdsp )
{
	// Simply add all open eventdev fds to the fd_set for select.
	int	i, j;
	FD_ZERO ( fdsp );
	j = -1;
	for ( i = 0; i < MAXEVDEVS; ++i )
	{
		if ( eventdevs[i] >= 0 )
		{
			FD_SET ( eventdevs[i], fdsp );
			if ( eventdevs[i] > j )
			{
				j = eventdevs[i];
			}
		}
	}
	return	j;
}

/*
 *	list_input_devices - Show a human-readable list of all input devices
 *	the current user has permissions to read from.
 *	Add info wether this probably can be "muted" in X11 if requested
 */
int	list_input_devices ()
{
	int	i, fd;
	char	buf[sizeof(EVDEVNAME)+8];
	struct input_id device_info;
	char	namebuf[256];
	char	*xinlist;
	FILE	*pf;
	char	*p, *q;
	char	x11 = 0;
	if ( NULL == ( xinlist = malloc ( 4096 ) ) )
	{
		printf ( "Memory alloc error\n" );
		return	1;
	}
	bzero ( xinlist, 4096 );
	if ( NULL != ( pf = popen ("xinput --list --name-only", "r" ) ) )
	{
		if ( 1 > fread ( xinlist, 1, 4095, pf ) )
		{
			printf ( "\tx11-mutable information not available.\n" );
		}
		fclose ( pf );
	}
	printf ( "List of available input devices:\n");
	printf ( "num\tVendor/Product, Name, -x compatible (x/-)\n" );
	for ( i = 0; i < MAXEVDEVS; ++i )
	{
		sprintf ( buf, EVDEVNAME, i );
		fd = open ( buf, O_RDONLY );
		if ( fd < 0 )
		{
			if ( errno == ENOENT ) { i = MAXEVDEVS ; break; }
			if ( errno == EACCES )
			{
				printf ( "%2d:\t[permission denied]\n", i );
			}
			continue;
		}
		if ( ioctl ( fd, EVIOCGID, &device_info ) < 0 )
		{
			close(fd); continue;
		}
		if ( ioctl ( fd, EVIOCGNAME(sizeof(namebuf)-4), namebuf+2) < 0 )
		{
			close(fd); continue;
		}
		namebuf[sizeof(namebuf)-4] = 0;
		x11 = 0;
		p = xinlist;
		while ( ( p != NULL ) && ( *p != 0 ) )
		{
			if ( NULL == ( q = strchr ( p, 0x0a ) ) ) { break; }
			*q = 0;
			if ( strcmp ( p, namebuf + 2 ) == 0 ) { x11 = 1; }
			*q = 0x0a;
			while ( (*q > 0) && (*q <= 0x20 ) ) { ++q; }
			p = q;
		}
		printf("%2d\t[%04hx:%04hx.%04hx] '%s' (%s)", i,
			device_info.vendor, device_info.product,
			device_info.version, namebuf + 2, x11 ? "+" : "-");
		printf("\n");
		close ( fd );
	}
	free ( xinlist );
	return	0;
}

/*	parse_events - At least one filedescriptor can now be read
 *	So retrieve data and parse it, eventually sending out a hid report!
 *	Return value <0 means connection broke and shall be disconnected
 */
int	parse_events ( fd_set * efds, int sockdesc )
{
	int	i, j;
	signed char	c;
	unsigned char	u;
	char	buf[sizeof(struct input_event)];
	char	hidrep[32]; // mouse ~6, keyboard ~11 chars
	struct input_event    * inevent = (void *)buf;
	struct hidrep_mouse_t * evmouse = (void *)hidrep;
	struct hidrep_keyb_t  * evkeyb  = (void *)hidrep;
	if ( efds == NULL ) { return -1; }
	for ( i = 0; i < MAXEVDEVS; ++i )
	{
		if ( 0 > eventdevs[i] ) continue;
		if ( ! ( FD_ISSET ( eventdevs[i], efds ) ) ) continue;
		j = read ( eventdevs[i], buf, sizeof(struct input_event) );
		if ( j == 0 )
		{
			if ( debugevents & 0x1 ) fprintf(stderr,".");
			continue;
		}
		if ( -1 == j )
		{
			if ( debugevents & 0x1 )
			{
				if ( errno > 0 )
				{
					fprintf(stderr,"%d|%d(%s) (expected %d bytes). ",eventdevs[i],errno,strerror(errno), (int)sizeof(struct input_event));
				}
				else
				{
					fprintf(stderr,"j=-1,errno<=0...");
				}
			}
			continue;
		}
		if ( sizeof(struct input_event) > j )
		{
			// exactly 24 on 64bit, (16 on 32bit): sizeof(struct input_event)
			//  chars expected != got: data invalid, drop it!
			continue;
		}
		fprintf(stderr,"   read(%d)from(%d)   ", j, i );
		if ( debugevents & 0x1 )
			fprintf ( stdout, "EVENT{%04X %04X %08X}\n", inevent->type,
			  inevent->code, inevent->value );
		switch ( inevent->type )
		{
		  case	EV_SYN:
			break;
		  case	EV_KEY:
			u = 1; // Modifier keys
			switch ( inevent->code )
			{
			  // *** Mouse button events
			  case	BTN_LEFT:
			  case	BTN_RIGHT:
			  case	BTN_MIDDLE:
				c = 1 << (inevent->code & 0x03);
				mousebuttons = mousebuttons & (0x07-c);
				if ( inevent->value == 1 )
				// Key has been pressed DOWN
				{
					mousebuttons=mousebuttons | c;
				}
				evmouse->btcode = 0xA1;
				evmouse->rep_id = REPORTID_MOUSE;
				evmouse->button = mousebuttons & 0x07;
				evmouse->axis_x =
				evmouse->axis_y =
				evmouse->axis_z = 0;
				if ( ! connectionok )
					break;
				j = send ( sockdesc, evmouse,
					sizeof(struct hidrep_mouse_t),
					MSG_NOSIGNAL );
				if ( 1 > j )
				{
					return	-1;
				}
				break;
			  // *** Special key: PAUSE
			  case	KEY_PAUSE:	
				// When pressed: abort connection
				if ( inevent->value == 0 )
				{
				    if ( connectionok )
				    {
					evkeyb->btcode=0xA1;
					evkeyb->rep_id=REPORTID_KEYBD;
					memset ( evkeyb->key, 0, 8 );
					evkeyb->modify = 0;
					j = send ( sockdesc, evkeyb,
					  sizeof(struct hidrep_keyb_t),
					  MSG_NOSIGNAL );
					close ( sockdesc );
				    }
				    // If also LCtrl+Alt pressed:
				    // Terminate program
				    if (( modifierkeys & 0x5 ) == 0x5 )
				    {
					return	-99;
				    }
				    return -1;
				}
				break;
			  // *** "Modifier" key events
			  case	KEY_RIGHTMETA:
				u <<= 1;
			  case	KEY_RIGHTALT:
				u <<= 1;
			  case	KEY_RIGHTSHIFT:
				u <<= 1;
			  case	KEY_RIGHTCTRL:
				u <<= 1;
			  case	KEY_LEFTMETA:
				u <<= 1;
			  case	KEY_LEFTALT:
				u <<= 1;
			  case	KEY_LEFTSHIFT:
				u <<= 1;
			  case	KEY_LEFTCTRL:
				evkeyb->btcode = 0xA1;
				evkeyb->rep_id = REPORTID_KEYBD;
				memcpy ( evkeyb->key, pressedkey, 8 );
				modifierkeys &= ( 0xff - u );
				if ( inevent->value >= 1 )
				{
					modifierkeys |= u;
				}
				evkeyb->modify = modifierkeys;
				j = send ( sockdesc, evkeyb,
					sizeof(struct hidrep_keyb_t),
					MSG_NOSIGNAL );
				if ( 1 > j )
				{
					return	-1;
				}
				break;
			  // *** Regular key events
			  case	KEY_KPDOT:	++u; // Keypad Dot ~ 99
			  case	KEY_KP0:	++u; // code 98...
			  case	KEY_KP9:	++u; // countdown...
			  case	KEY_KP8:	++u;
			  case	KEY_KP7:	++u;
			  case	KEY_KP6:	++u;
			  case	KEY_KP5:	++u;
			  case	KEY_KP4:	++u;
			  case	KEY_KP3:	++u;
			  case	KEY_KP2:	++u;
			  case	KEY_KP1:	++u;
			  case	KEY_KPENTER:	++u;
			  case	KEY_KPPLUS:	++u;
			  case	KEY_KPMINUS:	++u;
			  case	KEY_KPASTERISK:	++u;
			  case	KEY_KPSLASH:	++u;
			  case	KEY_NUMLOCK:	++u;
			  case	KEY_UP:		++u;
			  case	KEY_DOWN:	++u;
			  case	KEY_LEFT:	++u;
			  case	KEY_RIGHT:	++u;
			  case	KEY_PAGEDOWN:	++u;
			  case	KEY_END:	++u;
			  case	KEY_DELETE:	++u;
			  case	KEY_PAGEUP:	++u;
			  case	KEY_HOME:	++u;
			  case	KEY_INSERT:	++u;
						++u; //[Pause] key
						// - checked separately
			  case	KEY_SCROLLLOCK:	++u;
			  case	KEY_SYSRQ:	++u; //[printscr]
			  case	KEY_F12:	++u; //F12=> code 69
			  case	KEY_F11:	++u;
			  case	KEY_F10:	++u;
			  case	KEY_F9:		++u;
			  case	KEY_F8:		++u;
			  case	KEY_F7:		++u;
			  case	KEY_F6:		++u;
			  case	KEY_F5:		++u;
			  case	KEY_F4:		++u;
			  case	KEY_F3:		++u;
			  case	KEY_F2:		++u;
			  case	KEY_F1:		++u;
			  case	KEY_CAPSLOCK:	++u;
			  case	KEY_SLASH:	++u;
			  case	KEY_DOT:	++u;
			  case	KEY_COMMA:	++u;
			  case	KEY_GRAVE:	++u;
			  case	KEY_APOSTROPHE:	++u;
			  case	KEY_SEMICOLON:	++u;
			  case	KEY_102ND:	++u;
			  case	KEY_BACKSLASH:	++u;
			  case	KEY_RIGHTBRACE:	++u;
			  case	KEY_LEFTBRACE:	++u;
			  case	KEY_EQUAL:	++u;
			  case	KEY_MINUS:	++u;
			  case	KEY_SPACE:	++u;
			  case	KEY_TAB:	++u;
			  case	KEY_BACKSPACE:	++u;
			  case	KEY_ESC:	++u;
			  case	KEY_ENTER:	++u; //Return=> code 40
			  case	KEY_0:		++u;
			  case	KEY_9:		++u;
			  case	KEY_8:		++u;
			  case	KEY_7:		++u;
			  case	KEY_6:		++u;
			  case	KEY_5:		++u;
			  case	KEY_4:		++u;
			  case	KEY_3:		++u;
			  case	KEY_2:		++u;
			  case	KEY_1:		++u;
			  case	KEY_Z:		++u;
			  case	KEY_Y:		++u;
			  case	KEY_X:		++u;
			  case	KEY_W:		++u;
			  case	KEY_V:		++u;
			  case	KEY_U:		++u;
			  case	KEY_T:		++u;
			  case	KEY_S:		++u;
			  case	KEY_R:		++u;
			  case	KEY_Q:		++u;
			  case	KEY_P:		++u;
			  case	KEY_O:		++u;
			  case	KEY_N:		++u;
			  case	KEY_M:		++u;
			  case	KEY_L:		++u;
			  case	KEY_K:		++u;
			  case	KEY_J:		++u;
			  case	KEY_I:		++u;
			  case	KEY_H:		++u;
			  case	KEY_G:		++u;
			  case	KEY_F:		++u;
			  case	KEY_E:		++u;
			  case	KEY_D:		++u;
			  case	KEY_C:		++u;
			  case	KEY_B:		++u;
			  case	KEY_A:		u +=3;	// A =>  4
				evkeyb->btcode = 0xA1;
				evkeyb->rep_id = REPORTID_KEYBD;
				if ( inevent->value == 1 )
				{
					// "Key down": Add to list of
					// currently pressed keys
					for ( j = 0; j < 8; ++j )
					{
					    if (pressedkey[j] == 0)
					    {
						pressedkey[j]=u;
						j = 8;
					    }
					    else if(pressedkey[j] == u)
					    {
						j = 8;
					    }
					}
				}
				else if ( inevent->value == 0 )
				{	// KEY UP: Remove from array
					for ( j = 0; j < 8; ++j )
					{
					    if ( pressedkey[j] == u )
					    {
						while ( j < 7 )
						{
						    pressedkey[j] =
							pressedkey[j+1];
						    ++j;
						}
					    pressedkey[7] = 0;
					    }
					}
				} 
				else	// "Key repeat" event
				{
					; // This should be handled
					// by the remote side, not us.
				}
				memcpy ( evkeyb->key, pressedkey, 8 );
				evkeyb->modify = modifierkeys;
				if ( ! connectionok ) break;
				j = send ( sockdesc, evkeyb,
					sizeof(struct hidrep_keyb_t),
					MSG_NOSIGNAL );
				if ( 1 > j )
				{
					// If sending data fails,
					// abort connection
					return	-1;
				}
				break;
			  default:
				// Unknown key usage - ignore that
				;
			}
			break;
		  // *** Mouse movement events
		  case	EV_REL:
			switch ( inevent->code )
			{
			  case	ABS_X:
			  case	ABS_Y:
			  case	ABS_Z:
			  case	REL_WHEEL:
				evmouse->btcode = 0xA1;
				evmouse->rep_id = REPORTID_MOUSE;
				evmouse->button = mousebuttons & 0x07;
				evmouse->axis_x =
					( inevent->code == ABS_X ?
					  inevent->value : 0 );
				evmouse->axis_y =
					( inevent->code == ABS_Y ?
					  inevent->value : 0 );
				evmouse->axis_z =
					( inevent->code >= ABS_Z ?
					  inevent->value : 0 );
				if ( ! connectionok ) break;
				j = send ( sockdesc, evmouse,
					sizeof(struct hidrep_mouse_t),
					MSG_NOSIGNAL );
				if ( 1 > j )
				{
					return	-1;
				}
				break;
			}
			break;
		  // *** Various events we do not know. Ignore those.
		  case	EV_ABS:
		  case	EV_MSC:
		  case	EV_LED:
		  case	EV_SND:
		  case	EV_REP:
		  case	EV_FF:
		  case	EV_PWR:
		  case	EV_FF_STATUS:
			break;
		}
	}
	return	0;
}

int	main ( int argc, char ** argv )
{
	int			i,  j;
	int			sockint, sockctl; // For the listening sockets
	struct sockaddr_l2	l2a;
	socklen_t		alen=sizeof(l2a);
	int			sint,  sctl;	  // For the one-session-only
						  // socket descriptor handles
	char			badr[40];
	fd_set			fds;		  // fds for listening sockets
	fd_set			efds;	          // dev-event file descriptors
	int			maxevdevfileno;
	char			skipsdp = 0;	  // On request, disable SDPreg
	struct timeval		tv;		  // Used for "select"
	int			onlyoneevdev = -1;// If restricted to using only one evdev
	int			mutex11 = 0;      // try to "mute" in x11?
	char			*fifoname = NULL; // Filename for fifo, if applicable
	// Parse command line
	for ( i = 1; i < argc; ++i )
	{
		if ( ( 0 == strcmp ( argv[i], "-h"     ) ) ||
		     ( 0 == strcmp ( argv[i], "-?"     ) ) ||
		     ( 0 == strcmp ( argv[i], "--help" ) ) )
		{
			showhelp();
			return	0;
		}
		else if ( ( 0 == strcmp ( argv[i], "-s" ) ) ||
			  ( 0 == strcmp ( argv[i], "--skipsdp" ) ) )
		{
			skipsdp = 1;
		}
		else if ( 0 == strncmp ( argv[i], "-e", 2 ) ) {
			onlyoneevdev = atoi(argv[i]+2);
		}
		else if ( 0 == strcmp ( argv[i], "-l" ) )
		{
			return list_input_devices();
		}
		else if ( 0 == strcmp ( argv[i], "-d" ) )
		{
			debugevents = 0xffff;
		}
		else if ( 0 == strcmp ( argv[i], "-x" ) )
		{
			mutex11 = 1;
		}
		else if ( 0 == strncmp ( argv[i], "-f", 2 ) )
		{
			fifoname = argv[i] + 2;
		}
		else
		{
			fprintf ( stderr, "Invalid argument: \'%s\'\n", argv[i]);
			return	1;
		}
	}
	if ( ! skipsdp )
	{
		if ( dosdpregistration() )
		{
			fprintf(stderr,"Failed to register with SDP server\n");
			return	1;
		}
	}
	if ( NULL == fifoname )
	{
		if ( 1 > initevents (onlyoneevdev, mutex11) )
		{
			fprintf ( stderr, "Failed to open event interface files\n" );
			return	2;
		}
	} else {
		if ( 1 > initfifo ( fifoname ) )
		{
			fprintf ( stderr, "Failed to create/open fifo [%s]\n", fifoname );
			return	2;
		}
	}
	maxevdevfileno = add_filedescriptors ( &efds );
	if ( maxevdevfileno <= 0 )
	{
		fprintf ( stderr, "Failed to organize event input.\n" );
		return	13;
	}
	sockint = socket ( AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP );
	sockctl = socket ( AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP );
	if ( ( 0 > sockint ) || ( 0 > sockctl ) )
	{
		fprintf ( stderr, "Failed to generate bluetooth sockets\n" );
		return	2;
	}
	if ( btbind ( sockint, PSMHIDINT ) || btbind ( sockctl, PSMHIDCTL ))
	{
		fprintf ( stderr, "Failed to bind sockets (%d/%d) "
				"to PSM (%d/%d)\n",
				sockctl, sockint, PSMHIDCTL, PSMHIDINT );
		return	3;
	}
	if ( listen ( sockint, 1 ) || listen ( sockctl, 1 ) )
	{
		fprintf ( stderr, "Failed to listen on int/ctl BT socket\n" );
		close ( sockint );
		close ( sockctl );
		return	4;
	}
	// Add handlers to catch signals:
	// All do the same, terminate the program safely
	// Ctrl+C will be ignored though (SIGINT) while a connection is active
	signal ( SIGHUP,  &onsignal );
	signal ( SIGTERM, &onsignal );
	signal ( SIGINT,  &onsignal );
	fprintf ( stdout, "The HID-Client is now ready to accept connections "
			"from another machine\n" );
	//i = system ( "stty -echo" );	// Disable key echo to the console
	while ( 0 == prepareshutdown )
	{	// Wait for any shutdown-event to occur
		sint = sctl = 0;
		add_filedescriptors ( &efds );
		tv.tv_sec  = 0;
		tv.tv_usec = 0;
		while ( 0 < (j = select(maxevdevfileno+1,&efds,NULL,NULL,&tv)))
		{	// Collect and discard input data as long as available
			if ( -1 >  ( j = parse_events ( &efds, 0 ) ) )
			{	// LCtrl-LAlt-PAUSE - terminate program
				prepareshutdown = 1;
				break;
			}
			add_filedescriptors ( &efds );
			tv.tv_sec  = 0;
			tv.tv_usec = 500; // minimal delay
		}
		if ( prepareshutdown )
			break;
		connectionok = 0;
		tv.tv_sec  = 1;
		tv.tv_usec = 0;
		FD_ZERO ( &fds );
		FD_SET  ( sockctl, &fds );
		j = select ( sockctl + 1, &fds, NULL, NULL, &tv );
		if ( j < 0 )
		{
			if ( errno == EINTR )
			{	// Ctrl+C ? - handle that elsewhere
				continue;
			}
			fprintf ( stderr, "select() error on BT socket: %s! "
					"Aborting.\n", strerror ( errno ) );
			return	11;
		}
		if ( j == 0 )
		{	// Nothing happened, check for shutdown req and retry
			if ( debugevents & 0x2 )
				fprintf ( stdout, "," );
			continue;
		}
		sctl = accept ( sockctl, (struct sockaddr *)&l2a, &alen );
		if ( sctl < 0 )
		{
			if ( errno == EAGAIN )
			{
				continue;
			}
			fprintf ( stderr, "Failed to get a control connection:"
					" %s\n", strerror ( errno ) );
			continue;
		}
		tv.tv_sec  = 3;
		tv.tv_usec = 0;
		FD_ZERO ( &fds );
		FD_SET  ( sockint, &fds );
		j = select ( sockint + 1, &fds, NULL, NULL, &tv );
		if ( j < 0 )
		{
			if ( errno == EINTR )
			{	// Might have been Ctrl+C
				close ( sctl );
				continue;
			}
			fprintf ( stderr, "select() error on BT socket: %s! "
					"Aborting.\n", strerror ( errno ) );
			return	12;
		}
		if ( j == 0 )
		{
			fprintf ( stderr, "Interrupt connection failed to "
					"establish (control connection already"
					" there), timeout!\n" );
			close ( sctl );
			continue;
		}
		sint = accept ( sockint, (struct sockaddr *)&l2a, &alen );
		if ( sint < 0 )
		{
			close ( sctl );
			if ( errno == EAGAIN )
				continue;
			fprintf ( stderr, "Failed to get an interrupt "
					"connection: %s\n", strerror(errno));
			continue;
		}
		ba2str ( &l2a.l2_bdaddr, badr );
		badr[39] = 0;
		fprintf ( stdout, "Incoming connection from node [%s] "
				"accepted and established.\n", badr );
		tv.tv_sec  = 0;
		tv.tv_usec = 0;
		j = -1;
		add_filedescriptors ( &efds );
		while ( 0 < (j = select(maxevdevfileno+1,&efds,NULL,NULL,&tv)))
		{
			// This loop removes all input garbage that might be
			// already in the queue
			if ( -1 > ( j = parse_events ( &efds, 0 ) ) )
			{	// LCtrl-LAlt-PAUSE - terminate program
				prepareshutdown = 1;
				break;
			}
			add_filedescriptors ( &efds );
			tv.tv_sec  = 0;
			tv.tv_usec = 0;
		}
		if ( prepareshutdown )
			break;
		connectionok = 1;
		memset ( pressedkey, 0, 8 );
		modifierkeys = 0;
		mousebuttons = 0;
		while ( connectionok )
		{
			add_filedescriptors ( &efds );
			tv.tv_sec  = 1;
			tv.tv_usec = 0;
			while ( 0 < ( j = select ( maxevdevfileno + 1, &efds,
							NULL, NULL, &tv ) ) )
			{
				if ( 0 > ( j = parse_events ( &efds, sint ) ) )
				{
					// PAUSE pressed - close connection
					connectionok = 0;
					if ( j < -1 )
					{	// LCtrl-LAlt-PAUSE - terminate
						close ( sint );
						close ( sctl );
						prepareshutdown = 1;
					}
					break;
				}
				add_filedescriptors ( &efds );
				tv.tv_sec  = 1;
				tv.tv_usec = 0;
			}
		}
		connectionok = 0;
		close ( sint );
		close ( sctl );
		sint = sctl =  0;
		fprintf ( stderr, "Connection closed\n" );
		usleep ( 500000 ); // Sleep 0.5 secs between connections
				   // to not be flooded
	}
	//i = system ( "stty echo" );	   // Set console back to normal
	close ( sockint );
	close ( sockctl );
	if ( ! skipsdp )
	{
		sdpunregister ( sdphandle ); // Remove HID info from SDP server
	}
	if ( NULL == fifoname )
	{
		closeevents ();
	} else {
		closefifo ();
	}
	cleanup_stdin ();	   // And remove the input queue from stdin
	fprintf ( stderr, "Stopped hidclient.\n" );
	return	0;
}


void	showhelp ( void )
{
	fprintf ( stdout,
"hidclient  -  Virtual Bluetooth Mouse and Keyboard\n\n" \
"hidclient allows you to emulate a bluetooth HID device, based on the\n" \
"Bluez Bluetooth stack for Linux.\n\n" \
"The following command-line parameters can be used:\n" \
"-h|-?		Show this information\n" \
"-e<num>\t	Use only the one event device numbered <num>\n" \
"-f<name>	Use fifo <name> instead of event input devices\n" \
"-l		List available input devices\n" \
"-x		Disable device in X11 while hidclient is running\n" \
"-s|--skipsdp	Skip SDP registration\n" \
"		Do not register with the Service Discovery Infrastructure\n" \
"		(for debug purposes)\n\n" \
"Using hidclient in conjunction with \'openvt\' is recommended to minize\n" \
"impact of keystrokes meant to be transmitted to the local user interface\n" \
"(like running hidclient from a xterm window). You can make \'openvt\'\n" \
"spawn a text mode console, switch there and run hidclient with the\n" \
"following command line:\n" \
"		openvt -s -w hidclient\n" \
"This will even return to your xsession after hidclient terminates.\n\n" \
"hidclient connections can be dropped at any time by pressing the PAUSE\n" \
"key; the program will wait for other connections afterward.\n" \
"To stop hidclient, press LeftCtrl+LeftAlt+Pause.\n"
		);
	return;
}

void	onsignal ( int i )
{
	// Shutdown should be done if:
	switch ( i )
	{
	  case	SIGINT:
		if ( 0 == connectionok )
		{
			// No connection is active and Ctrl+C is pressed
			prepareshutdown = 2;
		}
		else
		{	// Ctrl+C ignored while connected
			// because it might be meant for the remote
			return;
		}
	  case	SIGTERM:
	  case	SIGHUP:
		// If a signal comes in
		prepareshutdown = 1;
		fprintf ( stderr, "Got shutdown request\n" );
		break;
	}
	return;
}

