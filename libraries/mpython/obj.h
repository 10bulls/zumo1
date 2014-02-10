// All Micro Python objects are at least this type
// It must be of pointer size

typedef machine_ptr_t mp_obj_t;
typedef machine_const_ptr_t mp_const_obj_t;

// Integers that fit in a pointer have this type
// (do we need to expose this in the public API?)

typedef machine_int_t mp_small_int_t;

// The machine floating-point type used for float and complex numbers

#if MICROPY_ENABLE_FLOAT
typedef machine_float_t mp_float_t;
#endif

// Anything that wants to be a Micro Python object must have
// mp_obj_base_t as its first member (except NULL and small ints)

struct _mp_obj_type_t;
struct _mp_obj_base_t {
    const struct _mp_obj_type_t *type;
};
typedef struct _mp_obj_base_t mp_obj_base_t;

// The NULL object is used to indicate the absence of an object
// It *cannot* be used when an mp_obj_t is expected, except where explicitly allowed

#define MP_OBJ_NULL ((mp_obj_t)NULL)

// These macros check for small int, qstr or object, and access small int and qstr values
//  - xxxx...xxx1: a small int, bits 1 and above are the value
//  - xxxx...xx10: a qstr, bits 2 and above are the value
//  - xxxx...xx00: a pointer to an mp_obj_base_t

// In SMALL_INT, next-to-highest bits is used as sign, so both must match for value in range
#define MP_OBJ_FITS_SMALL_INT(n) ((((n) ^ ((n) << 1)) & WORD_MSBIT_HIGH) == 0)
#define MP_OBJ_IS_SMALL_INT(o) ((((mp_small_int_t)(o)) & 1) != 0)
#define MP_OBJ_IS_QSTR(o) ((((mp_small_int_t)(o)) & 3) == 2)
#define MP_OBJ_IS_OBJ(o) ((((mp_small_int_t)(o)) & 3) == 0)
#define MP_OBJ_IS_TYPE(o, t) (MP_OBJ_IS_OBJ(o) && (((mp_obj_base_t*)(o))->type == (t))) // this does not work for checking a string, use below macro for that
#define MP_OBJ_IS_INT(o) (MP_OBJ_IS_SMALL_INT(o) || MP_OBJ_IS_TYPE(o, &int_type))
#define MP_OBJ_IS_STR(o) (MP_OBJ_IS_QSTR(o) || MP_OBJ_IS_TYPE(o, &str_type))

#define MP_OBJ_SMALL_INT_VALUE(o) (((mp_small_int_t)(o)) >> 1)
#define MP_OBJ_NEW_SMALL_INT(small_int) ((mp_obj_t)(((small_int) << 1) | 1))

#define MP_OBJ_QSTR_VALUE(o) (((mp_small_int_t)(o)) >> 2)
#define MP_OBJ_NEW_QSTR(qstr) ((mp_obj_t)((((machine_uint_t)qstr) << 2) | 2))

// These macros are used to declare and define constant function objects
// You can put "static" in front of the definitions to make them local

#define MP_DECLARE_CONST_FUN_OBJ(obj_name) extern const mp_obj_fun_native_t obj_name

#define MP_DEFINE_CONST_FUN_OBJ_VOID_PTR(obj_name, is_kw, n_args_min, n_args_max, fun_name) const mp_obj_fun_native_t obj_name = {{&fun_native_type}, is_kw, n_args_min, n_args_max, (void *)fun_name}
#define MP_DEFINE_CONST_FUN_OBJ_0(obj_name, fun_name) MP_DEFINE_CONST_FUN_OBJ_VOID_PTR(obj_name, false, 0, 0, (mp_fun_0_t)fun_name)
#define MP_DEFINE_CONST_FUN_OBJ_1(obj_name, fun_name) MP_DEFINE_CONST_FUN_OBJ_VOID_PTR(obj_name, false, 1, 1, (mp_fun_1_t)fun_name)
#define MP_DEFINE_CONST_FUN_OBJ_2(obj_name, fun_name) MP_DEFINE_CONST_FUN_OBJ_VOID_PTR(obj_name, false, 2, 2, (mp_fun_2_t)fun_name)
#define MP_DEFINE_CONST_FUN_OBJ_3(obj_name, fun_name) MP_DEFINE_CONST_FUN_OBJ_VOID_PTR(obj_name, false, 3, 3, (mp_fun_3_t)fun_name)
#define MP_DEFINE_CONST_FUN_OBJ_VAR(obj_name, n_args_min, fun_name) MP_DEFINE_CONST_FUN_OBJ_VOID_PTR(obj_name, false, n_args_min, (~((machine_uint_t)0)), (mp_fun_var_t)fun_name)
#define MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(obj_name, n_args_min, n_args_max, fun_name) MP_DEFINE_CONST_FUN_OBJ_VOID_PTR(obj_name, false, n_args_min, n_args_max, (mp_fun_var_t)fun_name)
#define MP_DEFINE_CONST_FUN_OBJ_KW(obj_name, n_args_min, fun_name) MP_DEFINE_CONST_FUN_OBJ_VOID_PTR(obj_name, true, n_args_min, (~((machine_uint_t)0)), (mp_fun_kw_t)fun_name)

// These macros are used to declare and define constant staticmethond and classmethod objects
// You can put "static" in front of the definitions to make them local

#define MP_DECLARE_CONST_STATICMETHOD_OBJ(obj_name) extern const mp_obj_staticmethod_t obj_name
#define MP_DECLARE_CONST_CLASSMETHOD_OBJ(obj_name) extern const mp_obj_classmethod_t obj_name

#define MP_DEFINE_CONST_STATICMETHOD_OBJ(obj_name, fun_name) const mp_obj_staticmethod_t obj_name = {{&mp_type_staticmethod}, fun_name}
#define MP_DEFINE_CONST_CLASSMETHOD_OBJ(obj_name, fun_name) const mp_obj_classmethod_t obj_name = {{&mp_type_classmethod}, fun_name}

// Need to declare this here so we are not dependent on map.h
struct _mp_map_t;
struct _mp_map_elem_t;
enum _mp_map_lookup_kind_t ENUM_SIZE_TYPE;

// Type definitions for methods

typedef mp_obj_t (*mp_fun_0_t)(void);
typedef mp_obj_t (*mp_fun_1_t)(mp_obj_t);
typedef mp_obj_t (*mp_fun_2_t)(mp_obj_t, mp_obj_t);
typedef mp_obj_t (*mp_fun_3_t)(mp_obj_t, mp_obj_t, mp_obj_t);
typedef mp_obj_t (*mp_fun_t)(void);
typedef mp_obj_t (*mp_fun_var_t)(uint n, const mp_obj_t *);
typedef mp_obj_t (*mp_fun_kw_t)(uint n, const mp_obj_t *, struct _mp_map_t *);

typedef enum {
    PRINT_STR, PRINT_REPR
} mp_print_kind_t;

typedef void (*mp_print_fun_t)(void (*print)(void *env, const char *fmt, ...), void *env, mp_obj_t o, mp_print_kind_t kind);
typedef mp_obj_t (*mp_make_new_fun_t)(mp_obj_t type_in, uint n_args, uint n_kw, const mp_obj_t *args);
typedef mp_obj_t (*mp_call_fun_t)(mp_obj_t fun, uint n_args, uint n_kw, const mp_obj_t *args);
typedef mp_obj_t (*mp_unary_op_fun_t)(int op, mp_obj_t);
typedef mp_obj_t (*mp_binary_op_fun_t)(int op, mp_obj_t, mp_obj_t);
typedef void (*mp_load_attr_fun_t)(mp_obj_t self_in, qstr attr, mp_obj_t *dest); // for fail, do nothing; for attr, dest[0] = value; for method, dest[0] = method, dest[1] = self
typedef bool (*mp_store_attr_fun_t)(mp_obj_t self_in, qstr attr, mp_obj_t value); // return true if store succeeded
typedef bool (*mp_store_item_fun_t)(mp_obj_t self_in, mp_obj_t index, mp_obj_t value); // return true if store succeeded

typedef struct _mp_method_t {
    const char *name;
    mp_const_obj_t fun;
} mp_method_t;

// Buffer protocol
typedef struct _buffer_info_t {
    // if we'd bother to support various versions of structure
    // (with different number of fields), we can distinguish
    // them with ver = sizeof(struct). Cons: overkill for *micro*?
    //int ver; // ?

    void *buf;
    machine_int_t len;

    // Rationale: have array.array and have SIMD operations on them
    // Cons: users can pass item size to processing functions themselves,
    // though that's not "plug&play"
    // int itemsize;

    // Rationale: to load arbitrary-sized sprites directly to LCD
    // Cons: a bit adhoc usecase
    // int stride;
} buffer_info_t;
#define BUFFER_READ  (1)
#define BUFFER_WRITE (2)
#define BUFFER_RW (BUFFER_READ | BUFFER_WRITE)
typedef struct _mp_buffer_p_t {
    machine_int_t (*get_buffer)(mp_obj_t obj, buffer_info_t *bufinfo, int flags);
} mp_buffer_p_t;

// Stream protocol
typedef struct _mp_stream_p_t {
    // On error, functions should return -1 and fill in *errcode (values are
    // implementation-dependent, but will be exposed to user, e.g. via exception).
    machine_int_t (*read)(mp_obj_t obj, void *buf, machine_uint_t size, int *errcode);
    machine_int_t (*write)(mp_obj_t obj, const void *buf, machine_uint_t size, int *errcode);
    // add seek() ?
} mp_stream_p_t;

struct _mp_obj_type_t {
    mp_obj_base_t base;
    const char *name;
    mp_print_fun_t print;
    mp_make_new_fun_t make_new;     // to make an instance of the type

    mp_call_fun_t call;
    mp_unary_op_fun_t unary_op;     // can return NULL if op not supported
    mp_binary_op_fun_t binary_op;   // can return NULL if op not supported

    mp_fun_1_t getiter;
    mp_fun_1_t iternext;

    // Alternatively, pointer(s) to interfaces to save space
    // in mp_obj_type_t at the expense of extra pointer and extra dereference
    // when actually used.
    mp_buffer_p_t buffer_p;
    mp_stream_p_t stream_p;

    const mp_method_t *methods;

    mp_load_attr_fun_t load_attr;
    mp_store_attr_fun_t store_attr;
    // Implements container[index] = val; note that load_item is implemented
    // by binary_op(RT_BINARY_OP_SUBSCR)
    mp_store_item_fun_t store_item;

    // these are for dynamically created types (classes)
    mp_obj_t bases_tuple;
    mp_obj_t locals_dict;

    /*
    What we might need to add here:

    store_subscr    list dict

    len             str tuple list map
    abs             float complex
    hash            bool int none str
    equal           int str
    less            int
    get_array_n     tuple list

    unpack seq      list tuple
    */
};

typedef struct _mp_obj_type_t mp_obj_type_t;

// Constant objects, globally accessible

extern const mp_obj_type_t mp_const_type;
extern const mp_obj_t mp_const_none;
extern const mp_obj_t mp_const_false;
extern const mp_obj_t mp_const_true;
extern const mp_obj_t mp_const_empty_tuple;
extern const mp_obj_t mp_const_ellipsis;
extern const mp_obj_t mp_const_stop_iteration; // special object indicating end of iteration (not StopIteration exception!)

// General API for objects

mp_obj_t mp_obj_new_type(const char *name, mp_obj_t bases_tuple, mp_obj_t locals_dict);
mp_obj_t mp_obj_new_none(void);
mp_obj_t mp_obj_new_bool(bool value);
mp_obj_t mp_obj_new_cell(mp_obj_t obj);
mp_obj_t mp_obj_new_int(machine_int_t value);
mp_obj_t mp_obj_new_int_from_uint(machine_uint_t value);
mp_obj_t mp_obj_new_int_from_long_str(const char *s);
mp_obj_t mp_obj_new_str(const byte* data, uint len, bool make_qstr_if_not_already);
mp_obj_t mp_obj_new_bytes(const byte* data, uint len);
#if MICROPY_ENABLE_FLOAT
mp_obj_t mp_obj_new_float(mp_float_t val);
mp_obj_t mp_obj_new_complex(mp_float_t real, mp_float_t imag);
#endif
mp_obj_t mp_obj_new_exception(qstr id);
mp_obj_t mp_obj_new_exception_msg(qstr id, const char *msg);
mp_obj_t mp_obj_new_exception_msg_1_arg(qstr id, const char *fmt, const char *a1);
mp_obj_t mp_obj_new_exception_msg_2_args(qstr id, const char *fmt, const char *a1, const char *a2);
mp_obj_t mp_obj_new_exception_msg_varg(qstr id, const char *fmt, ...); // counts args by number of % symbols in fmt, excluding %%; can only handle void* sizes (ie no float/double!)
mp_obj_t mp_obj_new_range(int start, int stop, int step);
mp_obj_t mp_obj_new_range_iterator(int cur, int stop, int step);
mp_obj_t mp_obj_new_fun_bc(int n_args, uint n_state, const byte *code);
mp_obj_t mp_obj_new_fun_asm(uint n_args, void *fun);
mp_obj_t mp_obj_new_gen_wrap(mp_obj_t fun);
mp_obj_t mp_obj_new_gen_instance(const byte *bytecode, uint n_state, int n_args, const mp_obj_t *args);
mp_obj_t mp_obj_new_closure(mp_obj_t fun, mp_obj_t closure_tuple);
mp_obj_t mp_obj_new_tuple(uint n, const mp_obj_t *items);
mp_obj_t mp_obj_new_list(uint n, mp_obj_t *items);
mp_obj_t mp_obj_new_dict(int n_args);
mp_obj_t mp_obj_new_set(int n_args, mp_obj_t *items);
mp_obj_t mp_obj_new_slice(mp_obj_t start, mp_obj_t stop, mp_obj_t step);
mp_obj_t mp_obj_new_bound_meth(mp_obj_t meth, mp_obj_t self);
mp_obj_t mp_obj_new_getitem_iter(mp_obj_t *args);
mp_obj_t mp_obj_new_module(qstr module_name);

mp_obj_type_t *mp_obj_get_type(mp_obj_t o_in);
const char *mp_obj_get_type_str(mp_obj_t o_in);

void mp_obj_print_helper(void (*print)(void *env, const char *fmt, ...), void *env, mp_obj_t o_in, mp_print_kind_t kind);
void mp_obj_print(mp_obj_t o, mp_print_kind_t kind);
void mp_obj_print_exception(mp_obj_t exc);

bool mp_obj_is_callable(mp_obj_t o_in);
machine_int_t mp_obj_hash(mp_obj_t o_in);
bool mp_obj_equal(mp_obj_t o1, mp_obj_t o2);
bool mp_obj_less(mp_obj_t o1, mp_obj_t o2);

machine_int_t mp_obj_get_int(mp_obj_t arg);
#if MICROPY_ENABLE_FLOAT
mp_float_t mp_obj_get_float(mp_obj_t self_in);
void mp_obj_get_complex(mp_obj_t self_in, mp_float_t *real, mp_float_t *imag);
#endif
//qstr mp_obj_get_qstr(mp_obj_t arg);
mp_obj_t *mp_obj_get_array_fixed_n(mp_obj_t o, machine_int_t n);
uint mp_get_index(const mp_obj_type_t *type, machine_uint_t len, mp_obj_t index);
mp_obj_t mp_obj_len_maybe(mp_obj_t o_in); /* may return NULL */

// none
extern const mp_obj_type_t none_type;

// bool
extern const mp_obj_type_t bool_type;
#define MP_BOOL(x) (x ? mp_const_true : mp_const_false)

// cell
mp_obj_t mp_obj_cell_get(mp_obj_t self_in);
void mp_obj_cell_set(mp_obj_t self_in, mp_obj_t obj);

// int
extern const mp_obj_type_t int_type;
// For long int, returns value truncated to machine_int_t
machine_int_t mp_obj_int_get(mp_obj_t self_in);
// Will rains exception if value doesn't fit into machine_int_t
machine_int_t mp_obj_int_get_checked(mp_obj_t self_in);

// exception
extern const mp_obj_type_t exception_type;
qstr mp_obj_exception_get_type(mp_obj_t self_in);
void mp_obj_exception_add_traceback(mp_obj_t self_in, qstr file, machine_uint_t line, qstr block);
void mp_obj_exception_get_traceback(mp_obj_t self_in, machine_uint_t *n, machine_uint_t **values);

// str
extern const mp_obj_type_t str_type;
mp_obj_t mp_obj_str_builder_start(const mp_obj_type_t *type, uint len, byte **data);
mp_obj_t mp_obj_str_builder_end(mp_obj_t o_in);
bool mp_obj_str_equal(mp_obj_t s1, mp_obj_t s2);
uint mp_obj_str_get_hash(mp_obj_t self_in);
uint mp_obj_str_get_len(mp_obj_t self_in);
qstr mp_obj_str_get_qstr(mp_obj_t self_in); // use this if you will anyway convert the string to a qstr
const char *mp_obj_str_get_str(mp_obj_t self_in); // use this only if you need the string to be null terminated
const byte *mp_obj_str_get_data(mp_obj_t self_in, uint *len);
void mp_str_print_quoted(void (*print)(void *env, const char *fmt, ...), void *env, const byte *str_data, uint str_len);

// bytes
extern const mp_obj_type_t bytes_type;

#if MICROPY_ENABLE_FLOAT
// float
extern const mp_obj_type_t float_type;
mp_float_t mp_obj_float_get(mp_obj_t self_in);
mp_obj_t mp_obj_float_binary_op(int op, mp_float_t lhs_val, mp_obj_t rhs);

// complex
extern const mp_obj_type_t complex_type;
void mp_obj_complex_get(mp_obj_t self_in, mp_float_t *real, mp_float_t *imag);
mp_obj_t mp_obj_complex_binary_op(int op, mp_float_t lhs_real, mp_float_t lhs_imag, mp_obj_t rhs_in);
#endif

// tuple
extern const mp_obj_type_t tuple_type;
void mp_obj_tuple_get(mp_obj_t self_in, uint *len, mp_obj_t **items);
void mp_obj_tuple_del(mp_obj_t self_in);

// list
extern const mp_obj_type_t list_type;
mp_obj_t mp_obj_list_append(mp_obj_t self_in, mp_obj_t arg);
void mp_obj_list_get(mp_obj_t self_in, uint *len, mp_obj_t **items);
void mp_obj_list_store(mp_obj_t self_in, mp_obj_t index, mp_obj_t value);
mp_obj_t mp_obj_list_sort(uint n_args, const mp_obj_t *args, struct _mp_map_t *kwargs);

// map (the python builtin, not the dict implementation detail)
extern const mp_obj_type_t map_type;

// enumerate
extern const mp_obj_type_t enumerate_type;

// filter
extern const mp_obj_type_t filter_type;

// dict
extern const mp_obj_type_t dict_type;
uint mp_obj_dict_len(mp_obj_t self_in);
mp_obj_t mp_obj_dict_store(mp_obj_t self_in, mp_obj_t key, mp_obj_t value);
struct _mp_map_t *mp_obj_dict_get_map(mp_obj_t self_in);

// set
extern const mp_obj_type_t set_type;
void mp_obj_set_store(mp_obj_t self_in, mp_obj_t item);

// slice
extern const mp_obj_type_t slice_type;
void mp_obj_slice_get(mp_obj_t self_in, machine_int_t *start, machine_int_t *stop, machine_int_t *step);

// zip
extern const mp_obj_type_t zip_type;

// array
extern const mp_obj_type_t array_type;
uint mp_obj_array_len(mp_obj_t self_in);
mp_obj_t mp_obj_new_bytearray_by_ref(uint n, void *items);

// functions
typedef struct _mp_obj_fun_native_t { // need this so we can define const objects (to go in ROM)
    mp_obj_base_t base;
    bool is_kw : 1;
    machine_uint_t n_args_min : (sizeof(machine_uint_t) - 1); // inclusive
    machine_uint_t n_args_max; // inclusive
    void *fun;
    // TODO add mp_map_t *globals
    // for const function objects, make an empty, const map
    // such functions won't be able to access the global scope, but that's probably okay
} mp_obj_fun_native_t;

extern const mp_obj_type_t fun_native_type;
extern const mp_obj_type_t fun_bc_type;
void mp_obj_fun_bc_get(mp_obj_t self_in, int *n_args, uint *n_state, const byte **code);

mp_obj_t mp_identity(mp_obj_t self);

// generator
extern const mp_obj_type_t gen_instance_type;

// module
extern const mp_obj_type_t module_type;
mp_obj_t mp_obj_new_module(qstr module_name);
mp_obj_t mp_obj_module_get(qstr module_name);
struct _mp_map_t *mp_obj_module_get_globals(mp_obj_t self_in);

// staticmethod and classmethod types; defined here so we can make const versions

extern const mp_obj_type_t mp_type_staticmethod;
extern const mp_obj_type_t mp_type_classmethod;

typedef struct _mp_obj_staticmethod_t {
    mp_obj_base_t base;
    mp_obj_t fun;
} mp_obj_staticmethod_t;

typedef struct _mp_obj_classmethod_t {
    mp_obj_base_t base;
    mp_obj_t fun;
} mp_obj_classmethod_t;

// sequence helpers
void mp_seq_multiply(const void *items, uint item_sz, uint len, uint times, void *dest);
