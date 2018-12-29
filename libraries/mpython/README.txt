The files were copied from the micropython repository and should be copied into the arduino\libraries\mpython folder.

Manual changes required:

Edit qstr.h to remove the build/py/ prefix from qstrdefs.generated.h

obj.h
added ENUM_SIZE_TYPE to enum _mp_map_lookup_kind_t 
This is to prevent use of enum _mp_map_lookup_kind_t without previous declaration error when
using header file from c++

map.h
typedef enum _mp_map_lookup_kind_t : ENUM_SIZE_TYPE {...

ENUM_SIZE_TYPE is defined in mpython as...

#ifdef __cplusplus
extern "C" {
#define ENUM_SIZE_TYPE : machine_int_t
#else
#define ENUM_SIZE_TYPE
#endif
