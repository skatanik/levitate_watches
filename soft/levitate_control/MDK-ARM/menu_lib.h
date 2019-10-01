#include "stdint.h"

typedef struct{
	char * parent;
	char ** child_slice;
	uint8_t childs_number;
	char [30] name;
	uint8_t param_is_bin;
	uint8_t bin_param;
	float   float_param;
	char * pointer_to_param;
	
}menu_item;

int add_menu_slice(char * parentPtr, char * slice_name, uint8_t slice_name_size);
