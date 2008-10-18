/****************************************************************************
 * Final Burn Alpha GX 0.1 Nintendo Wii/Gamecube Port
 *
 * softdev July 2006
 * Michniewski 2008
 * Tantric September 2008
 * Cthulhu32 October 2008
 *
 * unzip.h
 *
 * File unzip routines
 ****************************************************************************/

#include <gccore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

#include "fbagx.h"
#include "dvd.h"
#include "smbop.h"
#include "fileop.h"
#include "video.h"
#include "menudraw.h"
#include "unzip.h"

/*
  * PKWare Zip Header - adopted into zip standard
  */
#define PKZIPID 0x504b0304
#define MAXROM 0x500000
#define ZIPCHUNK 2048

/*
 * Zip files are stored little endian
 * Support functions for short and int types
 */
u32
FLIP32 (u32 b)
{
	unsigned int c;

	c = (b & 0xff000000) >> 24;
	c |= (b & 0xff0000) >> 8;
	c |= (b & 0xff00) << 8;
	c |= (b & 0xff) << 24;

	return c;
}

u16
FLIP16 (u16 b)
{
	u16 c;

	c = (b & 0xff00) >> 8;
	c |= (b & 0xff) << 8;

	return c;
}

/****************************************************************************
 * IsZipFile
 *
 * Returns TRUE when PKZIPID is first four characters of buffer
 ****************************************************************************/
int
IsZipFile (char *buffer)
{
	unsigned int *check;

	check = (unsigned int *) buffer;

	if (check[0] == PKZIPID)
		return 1;

	return 0;
}

 /*****************************************************************************
 * UnZipBuffer
 *
 * It should be noted that there is a limit of 5MB total size for any ROM
 ******************************************************************************/

int
UnZipBuffer (unsigned char *outbuffer, int method)
{
	PKZIPHEADER pkzip;
	int zipoffset = 0;
	int zipchunk = 0;
	char out[ZIPCHUNK];
	z_stream zs;
	int res;
	int bufferoffset = 0;
	int readoffset = 0;
	int have = 0;
	char readbuffer[ZIPCHUNK];
	u64 discoffset = 0;

	// Read Zip Header
	switch (method)
	{
		case METHOD_SD:
		case METHOD_USB:
			fseek(fatfile, 0, SEEK_SET);
			fread (readbuffer, 1, ZIPCHUNK, fatfile);
			break;

		case METHOD_DVD: // DVD
			discoffset = dvddir;
			dvd_read (readbuffer, ZIPCHUNK, discoffset);
			break;

		case METHOD_SMB: // From SMB
			SMB_ReadFile(readbuffer, ZIPCHUNK, 0, smbfile);
			break;
	}

	/*** Copy PKZip header to local, used as info ***/
	memcpy (&pkzip, readbuffer, sizeof (PKZIPHEADER));

	pkzip.uncompressedSize = FLIP32 (pkzip.uncompressedSize);

	ShowProgress ((char *)"Loading...", 0, pkzip.uncompressedSize);

	/*** Prepare the zip stream ***/
	memset (&zs, 0, sizeof (z_stream));
	zs.zalloc = Z_NULL;
	zs.zfree = Z_NULL;
	zs.opaque = Z_NULL;
	zs.avail_in = 0;
	zs.next_in = Z_NULL;
	res = inflateInit2 (&zs, -MAX_WBITS);

	if (res != Z_OK)
		return 0;

	/*** Set ZipChunk for first pass ***/
	zipoffset =
	(sizeof (PKZIPHEADER) + FLIP16 (pkzip.filenameLength) +
	FLIP16 (pkzip.extraDataLength));
	zipchunk = ZIPCHUNK - zipoffset;

	/*** Now do it! ***/
	do
	{
		zs.avail_in = zipchunk;
		zs.next_in = (Bytef *) & readbuffer[zipoffset];

		/*** Now inflate until input buffer is exhausted ***/
		do
		{
			zs.avail_out = ZIPCHUNK;
			zs.next_out = (Bytef *) & out;

			res = inflate (&zs, Z_NO_FLUSH);

			if (res == Z_MEM_ERROR)
			{
				inflateEnd (&zs);
				return 0;
			}

			have = ZIPCHUNK - zs.avail_out;
			if (have)
			{
				/*** Copy to normal block buffer ***/
				memcpy (&outbuffer[bufferoffset], &out, have);
				bufferoffset += have;
			}
		}
		while (zs.avail_out == 0);

		// Readup the next 2k block
		zipoffset = 0;
		zipchunk = ZIPCHUNK;

		switch (method)
		{
			case METHOD_SD:
			case METHOD_USB:
				fread (readbuffer, 1, ZIPCHUNK, fatfile);
				break;

			case METHOD_DVD:
				readoffset += ZIPCHUNK;
				dvd_read (readbuffer, ZIPCHUNK, discoffset+readoffset);
				break;

			case METHOD_SMB: // From SMB
				readoffset += ZIPCHUNK;
				SMB_ReadFile(readbuffer, ZIPCHUNK, readoffset, smbfile);
				break;
		}
		ShowProgress ((char *)"Loading...", bufferoffset, pkzip.uncompressedSize);
	}
	while (res != Z_STREAM_END);

	inflateEnd (&zs);

	if (res == Z_STREAM_END)
	{
		if (pkzip.uncompressedSize == (u32) bufferoffset)
			return bufferoffset;
		else
			return pkzip.uncompressedSize;
	}

	return 0;
}

/****************************************************************************
 * GetFirstZipFilename
 *
 * Returns the filename of the first file in the zipped archive
 * The idea here is to do the least amount of work required
 ***************************************************************************/

char *
GetFirstZipFilename (int method)
{
	char * firstFilename = NULL;
	char tempbuffer[ZIPCHUNK];

	// read start of ZIP
	switch (method)
	{
		case METHOD_SD:	// SD Card
		case METHOD_USB: // USB
			LoadFATFile (tempbuffer, ZIPCHUNK);
			break;

		case METHOD_DVD: // DVD
			LoadDVDFile ((unsigned char *)tempbuffer, ZIPCHUNK);
			break;

		case METHOD_SMB: // From SMB
			LoadSMBFile (tempbuffer, ZIPCHUNK);
			break;
	}

	tempbuffer[28] = 0; // truncate - filename length is 2 bytes long (bytes 26-27)
	int namelength = tempbuffer[26]; // filename length starts 26 bytes in

	firstFilename = &tempbuffer[30]; // first filename of a ZIP starts 31 bytes in
	firstFilename[namelength] = 0; // truncate at filename length

	return firstFilename;
}
