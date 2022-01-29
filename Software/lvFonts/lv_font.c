// Copyright (c) 2020 LVGL LLC
// MIT licence
// Source: https://github.com/lvgl/lvgl

/**
 * @file lv_font.c
 *
 */

#include "lv_font.h"

/** Searches base[0] to base[n - 1] for an item that matches *key.
 *
 * @note The function cmp must return negative if its first
 *  argument (the search key) is less that its second (a table entry),
 *  zero if equal, and positive if greater.
 *
 *  @note Items in the array must be in ascending order.
 *
 * @param key    Pointer to item being searched for
 * @param base   Pointer to first element to search
 * @param n      Number of elements
 * @param size   Size of each element
 * @param cmp    Pointer to comparison function (see #lv_font_codeCompare as a comparison function
 * example)
 *
 * @return a pointer to a matching item, or NULL if none exists.
 */
static void *_lv_utils_bsearch(const void *key, const void *base, uint32_t n, uint32_t size, int32_t (*cmp)(const void *pRef, const void *pElement)) {
	const char *middle;
	int32_t c;

	for (middle = base; n != 0;) {
		middle += (n / 2) * size;
		if ((c = (*cmp)(key, middle)) > 0) {
			n = (n / 2) - ((n & 1) == 0);
			base = (middle += size);
		} else if (c < 0) {
			n /= 2;
			middle = base;
		} else {
			return (char *)middle;
		}
	}
	return NULL;
}

static int32_t kern_pair_8_compare(const void *ref, const void *element) {
	const uint8_t *ref8_p = ref;
	const uint8_t *element8_p = element;

	/*If the MSB is different it will matter. If not return the diff. of the LSB*/
	if (ref8_p[0] != element8_p[0])
		return (int32_t)ref8_p[0] - element8_p[0];
	else
		return (int32_t)ref8_p[1] - element8_p[1];
}

static int32_t kern_pair_16_compare(const void *ref, const void *element) {
	const uint16_t *ref16_p = ref;
	const uint16_t *element16_p = element;

	/*If the MSB is different it will matter. If not return the diff. of the LSB*/
	if (ref16_p[0] != element16_p[0])
		return (int32_t)ref16_p[0] - element16_p[0];
	else
		return (int32_t)ref16_p[1] - element16_p[1];
}

/** Code Comparator.
 *
 *  Compares the value of both input arguments.
 *
 *  @param[in]  pRef        Pointer to the reference.
 *  @param[in]  pElement    Pointer to the element to compare.
 *
 *  @return Result of comparison.
 *  @retval < 0   Reference is greater than element.
 *  @retval = 0   Reference is equal to element.
 *  @retval > 0   Reference is less than element.
 *
 */
static int32_t unicode_list_compare(const void *ref, const void *element) {
	return ((int32_t)(*(uint16_t *)ref)) - ((int32_t)(*(uint16_t *)element));
}

static int8_t get_kern_value(const lv_font_t *font, uint32_t gid_left, uint32_t gid_right) {
	lv_font_fmt_txt_dsc_t *fdsc = (lv_font_fmt_txt_dsc_t *)font->dsc;

	int8_t value = 0;

	if (fdsc->kern_classes == 0) {
		/*Kern pairs*/
		const lv_font_fmt_txt_kern_pair_t *kdsc = fdsc->kern_dsc;
		if (kdsc->glyph_ids_size == 0) {
			/* Use binary search to find the kern value.
             * The pairs are ordered left_id first, then right_id secondly. */
			const uint8_t *g_ids = kdsc->glyph_ids;
			uint16_t g_id_both = (gid_right << 8) + gid_left; /*Create one number from the ids*/
			uint8_t *kid_p = _lv_utils_bsearch(&g_id_both, g_ids, kdsc->pair_cnt, 2, kern_pair_8_compare);

			/*If the `g_id_both` were found get its index from the pointer*/
			if (kid_p) {
				lv_uintptr_t ofs = (lv_uintptr_t)(kid_p - g_ids);
				ofs = ofs >> 1; /*ofs is for pair, divide by 2 to refer as a single value*/
				value = kdsc->values[ofs];
			}
		} else if (kdsc->glyph_ids_size == 1) {
			/* Use binary search to find the kern value.
             * The pairs are ordered left_id first, then right_id secondly. */
			const uint16_t *g_ids = kdsc->glyph_ids;
			lv_uintptr_t g_id_both = (uint32_t)((uint32_t)gid_right << 8) + gid_left; /*Create one number from the ids*/
			uint8_t *kid_p = _lv_utils_bsearch(&g_id_both, g_ids, kdsc->pair_cnt, 4, kern_pair_16_compare);

			/*If the `g_id_both` were found get its index from the pointer*/
			if (kid_p) {
				lv_uintptr_t ofs = (lv_uintptr_t)(kid_p - (const uint8_t *)g_ids);
				ofs = ofs >> 4; /*ofs is 4 byte pairs, divide by 4 to refer as a single value*/
				value = kdsc->values[ofs];
			}

		} else {
			/*Invalid value*/
		}
	} else {
		/*Kern classes*/
		const lv_font_fmt_txt_kern_classes_t *kdsc = fdsc->kern_dsc;
		uint8_t left_class = kdsc->left_class_mapping[gid_left];
		uint8_t right_class = kdsc->right_class_mapping[gid_right];

		/* If class = 0, kerning not exist for that glyph
         * else got the value form `class_pair_values` 2D array*/
		if (left_class > 0 && right_class > 0) {
			value = kdsc->class_pair_values[(left_class - 1) * kdsc->right_class_cnt + (right_class - 1)];
		}
	}
	return value;
}

static uint32_t get_glyph_dsc_id(const lv_font_t *font, uint32_t letter) {
	if (letter == '\0') return 0;

	lv_font_fmt_txt_dsc_t *fdsc = (lv_font_fmt_txt_dsc_t *)font->dsc;

	/*Check the cache first*/
	if (letter == fdsc->last_letter) return fdsc->last_glyph_id;

	uint16_t i;
	for (i = 0; i < fdsc->cmap_num; i++) {
		/*Relative code point*/
		uint32_t rcp = letter - fdsc->cmaps[i].range_start;
		if (rcp > fdsc->cmaps[i].range_length) continue;
		uint32_t glyph_id = 0;
		if (fdsc->cmaps[i].type == LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY) {
			glyph_id = fdsc->cmaps[i].glyph_id_start + rcp;
		} else if (fdsc->cmaps[i].type == LV_FONT_FMT_TXT_CMAP_FORMAT0_FULL) {
			const uint8_t *gid_ofs_8 = fdsc->cmaps[i].glyph_id_ofs_list;
			glyph_id = fdsc->cmaps[i].glyph_id_start + gid_ofs_8[rcp];
		} else if (fdsc->cmaps[i].type == LV_FONT_FMT_TXT_CMAP_SPARSE_TINY) {
			uint16_t key = rcp;
			uint8_t *p = _lv_utils_bsearch(&key, fdsc->cmaps[i].unicode_list, fdsc->cmaps[i].list_length,
			                               sizeof(fdsc->cmaps[i].unicode_list[0]), unicode_list_compare);

			if (p) {
				lv_uintptr_t ofs = (lv_uintptr_t)(p - (uint8_t *)fdsc->cmaps[i].unicode_list);
				ofs = ofs >> 1; /*The list stores `uint16_t` so the get the index divide by 2*/
				glyph_id = fdsc->cmaps[i].glyph_id_start + ofs;
			}
		} else if (fdsc->cmaps[i].type == LV_FONT_FMT_TXT_CMAP_SPARSE_FULL) {
			uint16_t key = rcp;
			uint8_t *p = _lv_utils_bsearch(&key, fdsc->cmaps[i].unicode_list, fdsc->cmaps[i].list_length,
			                               sizeof(fdsc->cmaps[i].unicode_list[0]), unicode_list_compare);

			if (p) {
				lv_uintptr_t ofs = (lv_uintptr_t)(p - (uint8_t *)fdsc->cmaps[i].unicode_list);
				ofs = ofs >> 1; /*The list stores `uint16_t` so the get the index divide by 2*/
				const uint8_t *gid_ofs_16 = fdsc->cmaps[i].glyph_id_ofs_list;
				glyph_id = fdsc->cmaps[i].glyph_id_start + gid_ofs_16[ofs];
			}
		}

		/*Update the cache*/
		fdsc->last_letter = letter;
		fdsc->last_glyph_id = glyph_id;
		return glyph_id;
	}

	fdsc->last_letter = letter;
	fdsc->last_glyph_id = 0;
	return 0;
}

/**
 * Return with the bitmap of a font.
 * @param font_p pointer to a font
 * @param letter an UNICODE character code
 * @return  pointer to the bitmap of the letter
 */
const uint8_t *lv_font_get_glyph_bitmap(const lv_font_t *font_p, uint32_t letter) {
	return font_p->get_glyph_bitmap(font_p, letter);
}

/**
 * Get the descriptor of a glyph
 * @param font_p pointer to font
 * @param dsc_out store the result descriptor here
 * @param letter an UNICODE letter code
 * @return true: descriptor is successfully loaded into `dsc_out`.
 *         false: the letter was not found, no data is loaded to `dsc_out`
 */
bool lv_font_get_glyph_dsc(const lv_font_t *font_p, lv_font_glyph_dsc_t *dsc_out, uint32_t letter,
                           uint32_t letter_next) {
	return font_p->get_glyph_dsc(font_p, dsc_out, letter, letter_next);
}

/**
 * Get the width of a glyph with kerning
 * @param font pointer to a font
 * @param letter an UNICODE letter
 * @param unicode_letter_next the next letter after `letter`. Used for kerning
 * @return the width of the glyph
 */
uint16_t lv_font_get_glyph_width(const lv_font_t *font, uint32_t letter, uint32_t unicode_letter_next) {
	lv_font_glyph_dsc_t g;
	bool ret;
	ret = lv_font_get_glyph_dsc(font, &g, letter, unicode_letter_next);
	if (ret)
		return g.adv_w;
	else
		return 0;
}

/**
 * Used as `get_glyph_bitmap` callback in LittelvGL's native font format if the font is uncompressed.
 * @param font pointer to font
 * @param unicode_letter an unicode letter which bitmap should be get
 * @return pointer to the bitmap or NULL if not found
 */
const uint8_t *lv_font_get_bitmap_fmt_txt(const lv_font_t *font, uint32_t unicode_letter) {
	if (unicode_letter == '\t') unicode_letter = ' ';

	lv_font_fmt_txt_dsc_t *fdsc = (lv_font_fmt_txt_dsc_t *)font->dsc;
	uint32_t gid = get_glyph_dsc_id(font, unicode_letter);
	if (!gid) return NULL;

	const lv_font_fmt_txt_glyph_dsc_t *gdsc = &fdsc->glyph_dsc[gid];

	if (fdsc->bitmap_format == LV_FONT_FMT_TXT_PLAIN) {
		if (gdsc) return &fdsc->glyph_bitmap[gdsc->bitmap_index];
	} else {
		/* eventually handle compressed bitmap */
		return NULL;
	}
	/* if not returned earlier then the letter is not found in this font */
	return NULL;
}

/**
 * Used as `get_glyph_dsc` callback in LittelvGL's native font format if the font is uncompressed.
 * @param font pointer to font
 * @param dsc_out store the result descriptor here
 * @param unicode_letter an UNICODE letter code
 * @param unicode_letter_next the next letter after `letter`. Used for kerning
 * @return true: descriptor is successfully loaded into `dsc_out`.
 *         false: the letter was not found, no data is loaded to `dsc_out`
 */
bool lv_font_get_glyph_dsc_fmt_txt(const lv_font_t *font, lv_font_glyph_dsc_t *dsc_out, uint32_t unicode_letter, uint32_t unicode_letter_next) {
	bool is_tab = false;
	if (unicode_letter == '\t') {
		unicode_letter = ' ';
		is_tab = true;
	}

	lv_font_fmt_txt_dsc_t *fdsc = (lv_font_fmt_txt_dsc_t *)font->dsc;
	uint32_t gid = get_glyph_dsc_id(font, unicode_letter);
	if (!gid) return false;

	int8_t kvalue = 0;
	if (fdsc->kern_dsc) {
		uint32_t gid_next = get_glyph_dsc_id(font, unicode_letter_next);
		if (gid_next) {
			kvalue = get_kern_value(font, gid, gid_next);
		}
	}

	/*Put together a glyph dsc*/
	const lv_font_fmt_txt_glyph_dsc_t *gdsc = &fdsc->glyph_dsc[gid];

	int32_t kv = ((int32_t)((int32_t)kvalue * fdsc->kern_scale) >> 4);

	uint32_t adv_w = gdsc->adv_w;
	if (is_tab) adv_w *= 2;

	adv_w += kv;
	adv_w = (adv_w + (1 << 3)) >> 4;

	dsc_out->adv_w = adv_w;
	dsc_out->box_h = gdsc->box_h;
	dsc_out->box_w = gdsc->box_w;
	dsc_out->ofs_x = gdsc->ofs_x;
	dsc_out->ofs_y = gdsc->ofs_y;
	dsc_out->bpp = (uint8_t)fdsc->bpp;

	if (is_tab) dsc_out->box_w = dsc_out->box_w * 2;

	return true;
}
