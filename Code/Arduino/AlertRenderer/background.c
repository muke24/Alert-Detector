#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_BACKGROUND
#define LV_ATTRIBUTE_IMG_BACKGROUND
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_BACKGROUND uint8_t background_map[] = {
  0x00, 0x00, 0x00, 0xff, 	/*Color of index 0*/
  0x00, 0x00, 0x00, 0x00, 	/*Color of index 1*/

  0x00, 
};

const lv_img_dsc_t background = {
  .header.cf = LV_IMG_CF_INDEXED_1BIT,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 1,
  .header.h = 1,
  .data_size = 9,
  .data = background_map,
};
