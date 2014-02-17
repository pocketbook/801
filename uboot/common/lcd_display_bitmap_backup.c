int lcd_display_bitmap(ulong bmp_image, int x, int y)
{
#if !defined(CONFIG_MCC200)
	ushort *cmap = NULL;
#endif
	ushort *cmap_base = NULL;
	ushort i, j;
	uchar *fb;
	bmp_image_t *bmp=(bmp_image_t *)bmp_image;
	uchar *bmap;
	ushort padded_line;
	unsigned long width, height, byte_width;
	unsigned long pwidth = panel_info.vl_col;
	unsigned colors, bpix, bmp_bpix;
	unsigned long compression;
	 unsigned short rgb565=0,lutvalue=0;
#if defined(CONFIG_PXA250)
	struct pxafb_info *fbi = &panel_info.pxa;
#elif defined(CONFIG_MPC823)
	volatile immap_t *immr = (immap_t *) CONFIG_SYS_IMMR;
	volatile cpm8xx_t *cp = &(immr->im_cpm);
#endif
	printf("%s %s %d  \n",__FILE__,__func__,__LINE__);

	if (!((bmp->header.signature[0]=='B') &&
		(bmp->header.signature[1]=='M'))) {
		printf ("Error: no valid bmp image at %lx\n", bmp_image);
		return 1;
	}

	width = le32_to_cpu (bmp->header.width);
	height = le32_to_cpu (bmp->header.height);
	bmp_bpix = le16_to_cpu(bmp->header.bit_count);
	colors = 1 << bmp_bpix;
	compression = le32_to_cpu (bmp->header.compression);

	bpix = NBITS(panel_info.vl_bpix);

	if ((bpix != 1) && (bpix != 8) && (bpix != 16) && (bpix != 24)) {
		printf ("Error: %d bit/pixel mode, but BMP has %d bit/pixel\n",
			bpix, bmp_bpix);
		return 1;
	}

	printf("panel_info.vl_row =%d, panel_info.vl_col =%d, bpix =%d\n",panel_info.vl_row, panel_info.vl_col, bpix);
	printf("width = %d, height=%d,bmp_bpix =%d, colors=%d, compression=%d\n", width, height, bmp_bpix, colors,compression);

#if defined(CONFIG_BMP_24BPP)
	/* We support displaying 24bpp BMPs on 16bpp LCDs */
	if (bpix != bmp_bpix && (bmp_bpix != 24 || bpix != 16) &&
		(bmp_bpix != 8 || bpix != 16)) {
#else
	/* We support displaying 8bpp BMPs on 16bpp LCDs */
	if (bpix != bmp_bpix && (bmp_bpix != 8 || bpix != 16)) {
#endif
		printf ("Error: %d bit/pixel mode, but BMP has %d bit/pixel\n",
			bpix,
			le16_to_cpu(bmp->header.bit_count));
		return 1;
	}

	printf ("Display-bmp: %d x %d  with %d colors\n",
		(int)width, (int)height, (int)colors);
	//Jim_dream
#if 0

#if !defined(CONFIG_MCC200)
	/* MCC200 LCD doesn't need CMAP, supports 1bpp b&w only */
	if (bmp_bpix == 8) {
#if defined(CONFIG_PXA250)
		cmap = (ushort *)fbi->palette;
#elif defined(CONFIG_MPC823)
		cmap = (ushort *)&(cp->lcd_cmap[255*sizeof(ushort)]);
#elif !defined(CONFIG_ATMEL_LCD)
		cmap = panel_info.cmap;
#endif
		cmap_base = cmap;

		/* Set color map */
		for (i=0; i<colors; ++i) {
			bmp_color_table_entry_t cte = bmp->color_table[i];
#if !defined(CONFIG_ATMEL_LCD)
			ushort colreg =
				( ((cte.red)   << 8) & 0xf800) |
				( ((cte.green) << 3) & 0x07e0) |
				( ((cte.blue)  >> 3) & 0x001f) ;
#ifdef CONFIG_SYS_INVERT_COLORS
			*cmap = 0xffff - colreg;
#else
			*cmap = colreg;
#endif
#if defined(CONFIG_MPC823)
			cmap--;
#else
			cmap++;
#endif
#else /* CONFIG_ATMEL_LCD */
			lcd_setcolreg(i, cte.red, cte.green, cte.blue);
#endif
		}
	}
#endif

#endif

	/*
	 * BMP format for Monochrome assumes that the state of a
	 * pixel is described on a per Bit basis, not per Byte.
	 * So, in case of Monochrome BMP we should align widths
	 * on a byte boundary and convert them from Bit to Byte
	 * units.
	 * Probably, PXA250 and MPC823 process 1bpp BMP images in
	 * their own ways, so make the converting to be MCC200
	 * specific.
	 */
#if defined(CONFIG_MCC200)
	if (bpix==1)
	{
		width = ((width + 7) & ~7) >> 3;
		x     = ((x + 7) & ~7) >> 3;
		pwidth= ((pwidth + 7) & ~7) >> 3;
	}
#endif
	padded_line = (width&0x3) ? ((width&~0x3)+4) : (width);
        printf("padded_line =%d, \n",padded_line);
#ifdef CONFIG_SPLASH_SCREEN_ALIGN
	if (x == BMP_ALIGN_CENTER)
		x = max(0, (pwidth - width) / 2);
	else if (x < 0)
		x = max(0, pwidth - width + x + 1);

	if (y == BMP_ALIGN_CENTER)
		y = max(0, (panel_info.vl_row - height) / 2);
	else if (y < 0)
		y = max(0, panel_info.vl_row - height + y + 1);
#endif /* CONFIG_SPLASH_SCREEN_ALIGN */

	if ((x + width)>pwidth)
		width = pwidth - x;
	if ((y + height)>panel_info.vl_row)
		height = panel_info.vl_row - y;

	bmap = (uchar *)bmp + le32_to_cpu (bmp->header.data_offset);
	printf("x=%d,y=%d,width=%d,height=%d bpix=%d,\n",x,y,width,height,bpix);
	printf("lcd_line_length =%d,\n",lcd_line_length);

	fb   = (uchar *) (lcd_base + (y + height - 1) * lcd_line_length + x * bpix / 8);
	//fb   = (uchar *) (lcd_base + (y + height - 1) * lcd_line_length + x);
	printf("%s, fb=0x%08lx,bmap=0x%x\n",__func__,fb,bmap);

	switch (bmp_bpix) {
	case 1: /* pass through */
	case 8:
		if (bpix != 16)
			byte_width = width;
		else
			byte_width = width * 2;
#if defined(DEMO)
		for (i = 0; i < height; ++i) {
			WATCHDOG_RESET();
			for (j = 0; j < width; j++) {
				if (bpix != 16) {
#if defined(CONFIG_PXA250) || defined(CONFIG_ATMEL_LCD)
					*(fb++) = *(bmap++);
#elif defined(CONFIG_MPC823) || defined(CONFIG_MCC200)
					*(fb++) = 255 - *(bmap++);
#endif
				} else {
					*(uint16_t *)fb = cmap_base[*(bmap++)];
					fb += sizeof(uint16_t) / sizeof(*fb);
				}
			}
			bmap += (width - padded_line);
			fb   -= (byte_width + lcd_line_length);
		}
#elif defined(CONFIG_EPDC_RGB)
#if 0
		    for (i = 0; i < height; i++) {
		        for (j= 0; j < width; j++) {
		            rgb565 = *(bmap++);

		            //get R 
		            lutvalue = ((rgb565 >> 12) & 0xf) << 4;
		            *(fb++)  = (lutvalue << 8) | (lutvalue << 3) | (lutvalue >> 3);

		            //get G 
		            lutvalue = ((rgb565 >> 7) & 0xf) << 4;
		             *(fb++)  = (lutvalue << 8) | (lutvalue << 3) | (lutvalue >> 3);

		            //get B 
		            lutvalue = ((rgb565 >> 1) & 0xf) << 4;
		            *(fb++)  = (lutvalue << 8) | (lutvalue << 3) | (lutvalue >> 3);
		        }
		    }
			//bmap += (width - padded_line);
			//fb   -= (byte_width + lcd_line_length);

#else
		for (i = 0; i < height; ++i) {
			WATCHDOG_RESET();
			for (j = 0; j < width; j++) {
				if (bpix != 16) {
					*(fb++) = *(bmap++);
					//*(fb++) = 255 - *(bmap++);
				} else {
					*(uint16_t *)fb = cmap_base[*(bmap++)];
					fb += sizeof(uint16_t) / sizeof(*fb);
				}
			}
			bmap += (width - padded_line);
			fb   -= (byte_width + lcd_line_length);
		}		
#endif
#else
		for (i = 0; i < height; ++i) {
			for (j = 0; j < width; j++) {
				*(fb++) = *(bmap++);			
			}
			bmap += (width - padded_line);
			fb   -= (byte_width + lcd_line_length);
		}
#endif
		
		break;

#if defined(CONFIG_BMP_16BPP)
	case 16:
		for (i = 0; i < height; ++i) {
			WATCHDOG_RESET();
			for (j = 0; j < width; j++) {
#if defined(CONFIG_ATMEL_LCD_BGR555)
				*(fb++) = ((bmap[0] & 0x1f) << 2) |
					(bmap[1] & 0x03);
				*(fb++) = (bmap[0] & 0xe0) |
					((bmap[1] & 0x7c) >> 2);
				bmap += 2;
#else
				*(fb++) = *(bmap++);
				*(fb++) = *(bmap++);
#endif
			}
			bmap += (padded_line - width) * 2;
			fb   -= (width * 2 + lcd_line_length);
		}
		break;
#endif /* CONFIG_BMP_16BPP */
#if defined(CONFIG_BMP_24BPP)
	case 24:
		if (bpix != 16) {
			printf("Error: %d bit/pixel mode,"
				"but BMP has %d bit/pixel\n",
				bpix, bmp_bpix);
			break;
		}
		for (i = 0; i < height; ++i) {
			WATCHDOG_RESET();
			for (j = 0; j < width; j++) {
				*(uint16_t *)fb = ((*(bmap + 2) << 8) & 0xf800)
						| ((*(bmap + 1) << 3) & 0x07e0)
						| ((*(bmap) >> 3) & 0x001f);
				bmap += 3;
				fb += sizeof(uint16_t) / sizeof(*fb);
			}
			bmap += (width - padded_line);
			fb   -= ((2*width) + lcd_line_length);
		}
		break;
#endif /* CONFIG_BMP_24BPP */
	default:
		break;
	};

	return (0);
}