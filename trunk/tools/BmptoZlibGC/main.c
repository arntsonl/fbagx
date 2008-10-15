/*
    BMP To GameCube graphics converter.

    This program takes a BMP graphics file and outputs a
    Gamecube formatted C array into a .h file.
    This can then be included into a GC program and the data
    blitted directly to the frame buffer.


    Usage:  BmpToGC  [-MP 0xFF00FF 0x000000] <InputFileName.bmp>


    The -MP switch turns on Magic Pink processing
    which I personally use for transparency.
    When my graphics libraries see Pink RGB (255, 0, 255),
    I don't display the color.
    Since the GameCube stores two pixels per word, sometimes
    pink gets mixed with another color and my graphics library
    doesn't detect the transparency.  So the -MP switch will
    change a pink pixel to another color if it will get
    mixed.


    (c) 2007 PaceMaker - No guarantees, etc,
    May be freely copied, distributed, mangled, etc.
*/

#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <zlib.h>

#pragma pack (2)

typedef struct
{
   unsigned short int type;                 /* Magic identifier            */
   unsigned int size;                       /* File size in bytes          */
   unsigned short int reserved1, reserved2;
   unsigned int offset;                     /* Offset to image data, bytes */
} BMPHeader;


typedef struct {
   unsigned int size;               /* Header size in bytes      */
   int width,height;                /* Width and height of image */
   unsigned short int planes;       /* Number of colour planes   */
   unsigned short int bits;         /* Bits per pixel            */
   unsigned int compression;        /* Compression type          */
   unsigned int imagesize;          /* Image size in bytes       */
   int xresolution,yresolution;     /* Pixels per meter          */
   unsigned int ncolours;           /* Number of colours         */
   unsigned int importantcolours;   /* Important colours         */
} BMPInfoHeader;


typedef struct
{
    unsigned char b, g, r;
} BMPPixel;


int DumpHeader (BMPHeader * pHeader)
{
//    printf ("\tHeader: type=0x%hx, size=%d, offset=0x%x\n", pHeader->type, pHeader->size, pHeader->offset);

    if (pHeader->type != 0x4d42)
    {
        printf ("Header error: bad type (0x%x)\n", pHeader->type);
        return (0);
    }

    return (1);
}


int DumpInfoHeader (BMPInfoHeader * pIHeader)
{
    printf ("\tInfoHeader: header size=%d, Width, Height=(%d,%d)\n", pIHeader->size, pIHeader->width, pIHeader->height);
    printf ("\tInfoHeader: planes=%d, bits=%d, compression=%d, image size (bytes)=%d\n", pIHeader->planes, pIHeader->bits, pIHeader->compression, pIHeader->imagesize);
//    printf ("\tInfoHeader: XY Resolution(%d,%d), Num Colors=%d, ImportantColors=%d\n", pIHeader->xresolution, pIHeader->yresolution, pIHeader->ncolours, pIHeader->importantcolours);

    if (pIHeader->size != 40)
    {
        printf ("ERROR: Info Header error: bad header size (0x%d)\n", pIHeader->size);
        return (0);
    }

    if (pIHeader->width % 2)
    {
        printf ("ERROR: Info Header error: width is not a multiple of 2 (0x%d)\n", pIHeader->width);
        return (0);
    }

    if (pIHeader->compression != 0)
    {
        printf ("ERROR: Info Header error: compression not supported (0x%d)\n", pIHeader->compression);
        return (0);
    }

    if (pIHeader->ncolours != 0)
    {
        printf ("ERROR: Info Header error: indexed colors not supported (%d)\n", pIHeader->ncolours);
        return (0);
    }

    if (pIHeader->planes != 1)
    {
        printf ("ERROR: Info Header error: only one plane supported (%d)\n", pIHeader->planes);
        return (0);
    }

    if (pIHeader->bits != 24)
    {
        printf ("ERROR: Info Header error: only 24 bit color depth supported (%d)\n", pIHeader->bits);
        return (0);
    }


    return (1);
}

int DumpData (BMPInfoHeader * pIHeader, BMPPixel * pData)
{
int i, j;

    printf ("DataDump:\n");
    for (j = pIHeader->height - 1; j >= 0; j--)
    {
        for (i = 0; i < pIHeader->width; i++)
        {
           printf ("(%02x,%02x,%02x)",
            pData [j * pIHeader->width + i].r,
            pData [j * pIHeader->width + i].g,
            pData [j * pIHeader->width + i].b);
        }
        printf ("\n");
    }


    return (1);
}


unsigned int RGBtoGC (unsigned char r1, unsigned char g1, unsigned char b1, unsigned char r2, unsigned char g2, unsigned char b2)
{
  int y1, cb1, cr1, y2, cb2, cr2, cb, cr;

  y1 = (299 * r1 + 587 * g1 + 114 * b1) / 1000;
  cb1 = (-16874 * r1 - 33126 * g1 + 50000 * b1 + 12800000) / 100000;
  cr1 = (50000 * r1 - 41869 * g1 - 8131 * b1 + 12800000) / 100000;

  y2 = (299 * r2 + 587 * g2 + 114 * b2) / 1000;
  cb2 = (-16874 * r2 - 33126 * g2 + 50000 * b2 + 12800000) / 100000;
  cr2 = (50000 * r2 - 41869 * g2 - 8131 * b2 + 12800000) / 100000;

  cb = (cb1 + cb2) >> 1;
  cr = (cr1 + cr2) >> 1;

  return (y1 << 24) | (cb << 16) | (y2 << 8) | cr;
}


int Convert (char * szFileName, BMPInfoHeader * pIHeader, BMPPixel * pData, int bMagicPink, BMPPixel MPFrom, BMPPixel MPTo)
{
int i, j, gc, nIndex;
int bP1Pink, bP2Pink;
char szNewName [256];
char szUpperName [256];
char szLowerName [256];
char * p;
FILE * fp;
int err;
Byte* compr;
uLong comprLen = 500000*sizeof(int);
short * pixelArray;
unsigned int len = (pIHeader->width * pIHeader->height * 2);

    compr    = (Byte*)calloc((uInt)comprLen, 1);

    // Create new file name.
    sprintf (szNewName, "%s.h", szFileName);
    fp = fopen (szNewName, "wt");

    if (! fp)
    {
        printf("ERROR: error opening file: %s\n", szNewName);
        return (0);
    }

    printf ("Output File: %s\n", szNewName);

    // Create upper case name, no extension.
    strcpy (szUpperName, szFileName);
    strupr (szUpperName);
    p = strrchr (szUpperName, '.');
    if (p)
        *p = 0;

    // Create lower case name, no extension.  Not really lower case, just normal.
    strcpy (szLowerName, szFileName);
    p = strchr (szLowerName, '.');
    if (p)
        *p = 0;

    fprintf (fp, "/*\n" \
                 " * File  : %s\n" \
                 " * Width : %d\n" \
                 " * Height: %d\n" \
                 "*/\n", szFileName, pIHeader->width, pIHeader->height);
    fprintf (fp, "#ifndef _%s_H_ \n", szUpperName);
    fprintf (fp, "#define _%s_H_ \n", szUpperName);
    fprintf (fp, "#define %s_WIDTH      (%d)\n", szUpperName, pIHeader->width);
    fprintf (fp, "#define %s_HEIGHT     (%d)\n", szUpperName, pIHeader->height);
    fprintf (fp, "#define %s_SIZE       (%d)\n", szUpperName, len);


    pixelArray = (int*)malloc(len*sizeof(int));

    for (j = pIHeader->height - 1; j >= 0; j--)
    {
        for (i = 0; i < pIHeader->width; i += 2)
        {
 /*           if ((i != 0) && ((i % 16) == 0))
                fprintf (fp, "\n");
*/
            nIndex = j * pIHeader->width + i;


            // Magic pinks are used for transparency.
            // Since the GC stores two pixels in one 32bit word,
            // we check to ensure a pink pixel isn't getting mixed
            // into a pixel with real data.
            // We duplciate the real data over the pink pixel.
            if (bMagicPink)
            {
                bP1Pink = ((pData [nIndex].r == MPFrom.r) &&
                           (pData [nIndex].g == MPFrom.g) &&
                           (pData [nIndex].b == MPFrom.b));
                bP2Pink = ((pData [nIndex+1].r == MPFrom.r) &&
                           (pData [nIndex+1].g == MPFrom.g) &&
                           (pData [nIndex+1].b == MPFrom.b));

                if (bP1Pink && (! bP2Pink))
                {
                    pData [nIndex].r = MPTo.r;//pData [nIndex+1].r;
                    pData [nIndex].g = MPTo.g;//pData [nIndex+1].g;
                    pData [nIndex].b = MPTo.b;//pData [nIndex+1].b;
                }
                else if ((! bP1Pink) && bP2Pink)
                {
                    pData [nIndex+1].r = MPTo.r;//pData [nIndex].r;
                    pData [nIndex+1].g = MPTo.g;//pData [nIndex].g;
                    pData [nIndex+1].b = MPTo.b;//pData [nIndex].b;
                }
            }

            gc = RGBtoGC (
                pData [nIndex].r,
                pData [nIndex].g,
                pData [nIndex].b,
                pData [nIndex + 1].r,
                pData [nIndex + 1].g,
                pData [nIndex + 1].b);

            pixelArray[((pIHeader->height-1)-j) * pIHeader->width + (i)] = (((gc&0xFF000000)>>24) | ((gc&0x00FF0000)>>8));
            pixelArray[((pIHeader->height-1)-j) * pIHeader->width + (i+1)] = (((gc&0x0000FF00)>>8) | ((gc&0x000000FF)<<8));
            //fprintf (fp, "0x%08X, ", gc);

        }
        //fprintf (fp, "\n");
    }

    err = compress(compr, &comprLen, (const Bytef*)pixelArray, len);

    printf("Compressed to (bytes) - %i\n", (int)comprLen);

    fprintf (fp, "#define %s_COMPRESSED      (%i)\n\n", szUpperName, (int)comprLen);
    fprintf (fp, "const unsigned char %s_Bitmap[]={\n", szLowerName);

    for(i = 0; i < comprLen; i++){
        if (i == 0 ){ fprintf(fp,"  ");}
        else if ((i % 16) == 0){ fprintf(fp,"\n  ");}
        if (i == comprLen -1 ) {
            fprintf(fp, "0x%02x", compr[i]);
        }
        else {
            fprintf(fp, "0x%02x,", compr[i]);
        }
    }

    fprintf (fp, "\n};\n");
    fprintf (fp, "#endif // _%s_\n", szUpperName);

    fclose (fp);

    free(compr);
    free(pixelArray);

    return (1);
}



void PrintHelp (void)
{
    printf ("Args:  [-MP [0xFF00FF 0x000000]] <filename>\n");
    printf ("Converts a BMP file to GCube format with option 'Magic Pink' processing.\n");
    printf ("'Magic Pink' is a quick and dirty type of transparency support.");
}


int main (int argc, char * argv [])
{
FILE * fp;
char * pFileName;
int i, j, nRemainder;
unsigned char r,g,b;

BMPHeader sHeader;
BMPInfoHeader sInfoHeader;
BMPPixel * pData;

int nColor, nStatus;
int bMagicPink;         // 0 = Off, 1 = use From and To,  2 = Use From and copy color
BMPPixel MPFrom, MPTo;  // 'From' is the magic color.  'To' is what it gets translated to.

    printf ("==== BmpToZlibGC Converter, Version 1.1, 2008 Cthulhu32 ====\n\n");
    printf ("\tbased off of BmpToGC Converter, Version 1.0, (c) 2007 PaceMaker\n\n");


    // Check args
    if (argc < 2)
    {
        PrintHelp ();
        return (1);
    }



    if (argc == 2)
    {
        pFileName = argv [1];
        bMagicPink = 0;
    }
    else if (argc == 3)
    {
        bMagicPink = 1;
        pFileName = argv [2];
        MPFrom.r = 255;
        MPFrom.g = 0;
        MPFrom.b = 255;
        MPTo.r = 0;
        MPTo.g = 0;
        MPTo.b = 0;
    }
    else if (argc == 5)
    {
        bMagicPink = 1;
        pFileName = argv [4];


        // Parse 'From' Color.
        nStatus = sscanf (argv [2], "%i", & nColor);
        if (! nStatus)
        {
            PrintHelp ();
            return (1);
        }
        MPFrom.r = nColor >> 16;
        MPFrom.g = (nColor >> 8) & 0xFF;
        MPFrom.b = nColor & 0xFF;

        // Parse 'To' Color.
        nStatus = sscanf (argv [3], "%i", & nColor);
        if (! nStatus)
        {
            PrintHelp ();
            return (1);
        }
        MPTo.r = nColor >> 16;
        MPTo.g = (nColor >> 8) & 0xFF;
        MPTo.b = nColor & 0xFF;

    }
    else
    {
        PrintHelp ();
        return (1);
    }

    if (bMagicPink == 1)
        printf ("Magic Pink conversion is on: (%d,%d,%d) -> (%d,%d,%d).\n", MPFrom.r, MPFrom.g, MPFrom.b, MPTo.r, MPTo.g, MPTo.b);
    if (bMagicPink == 2)
        printf ("Magic Pink conversion is on: (%d,%d,%d) -> (Copy Color).\n", MPFrom.r, MPFrom.g, MPFrom.b);

    printf ("Opening File: %s\n", pFileName);

    // Open the file.
    fp = fopen (pFileName, "rb");
    if (! fp)
    {
        printf ("Can't open: %s\n", pFileName);
        return (2);
    }


    // Read the first header.
    nStatus = fread (& sHeader, sizeof (sHeader), 1, fp);
    if (nStatus != 1)
    {
        printf ("Can't read header: %d\n", nStatus);
        return (3);
    }

    if (! DumpHeader (& sHeader))
        return (3);

    // Read the second header
    nStatus = fread (& sInfoHeader, sizeof (sInfoHeader), 1, fp);
    if (nStatus != 1)
    {
        printf ("Can't read info header: %d\n", nStatus);
        return (4);
    }


    if (! DumpInfoHeader (& sInfoHeader))
        return (4);


    fseek (fp, sHeader.offset, SEEK_SET);


    pData = (BMPPixel *) malloc (sInfoHeader.height * sInfoHeader.width * sizeof (BMPPixel));

    for (j = 0; j < sInfoHeader.height; j++)
    {
        for (i = 0; i < sInfoHeader.width; i++)
        {

            if (fread(&b,sizeof(unsigned char),1,fp) != 1)
            {
               printf("Image read failed\n");
               return (5);
            }
            if (fread(&g,sizeof(unsigned char),1,fp) != 1)
            {
               printf("Image read failed\n");
               return (5);
            }
            if (fread(&r,sizeof(unsigned char),1,fp) != 1)
            {
               printf("Image read failed\n");
               return (5);
            }

            pData [j * sInfoHeader.width + i].r = r;
            pData [j * sInfoHeader.width + i].g = g;
            pData [j * sInfoHeader.width + i].b = b;
 //           printf ("(%02x,%02x,%02x)", r, g, b);
        } /* i */
//        printf("\n");

        // Each line must be a multiple of 4.
        for (nRemainder = 0; nRemainder < sInfoHeader.width % 4; nRemainder ++)
            fread(&g,sizeof(unsigned char),1,fp);
    } /* j */

    fclose (fp);

/*
    if (! DumpData (& sInfoHeader, pData))
    {
        return (6);
    }
*/

    Convert (pFileName, & sInfoHeader, pData, bMagicPink, MPFrom, MPTo);

    printf ("Done.\n");

    return (0);
}
