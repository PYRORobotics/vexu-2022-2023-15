/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7-dev */

#ifndef PB_MESSAGES_PB_H_INCLUDED
#define PB_MESSAGES_PB_H_INCLUDED
#include "nanopb/pb.h"
#include "nanopb/pb_encode.h"
#include "nanopb/pb_decode.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _Command {
    bool out_1;
    bool out_2;
    bool has_counter;
    int32_t counter;
} Command;

typedef struct _Status {
    bool in_1;
    bool in_2;
    bool has_counter;
    int32_t counter;
} Status;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define Command_init_default                     {0, 0, false, 0}
#define Status_init_default                      {0, 0, false, 0}
#define Command_init_zero                        {0, 0, false, 0}
#define Status_init_zero                         {0, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define Command_out_1_tag                        1
#define Command_out_2_tag                        2
#define Command_counter_tag                      3
#define Status_in_1_tag                          1
#define Status_in_2_tag                          2
#define Status_counter_tag                       3

/* Struct field encoding specification for nanopb */
#define Command_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, BOOL,     out_1,             1) \
X(a, STATIC,   REQUIRED, BOOL,     out_2,             2) \
X(a, STATIC,   OPTIONAL, INT32,    counter,           3)
#define Command_CALLBACK NULL
#define Command_DEFAULT NULL

#define Status_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, BOOL,     in_1,              1) \
X(a, STATIC,   REQUIRED, BOOL,     in_2,              2) \
X(a, STATIC,   OPTIONAL, INT32,    counter,           3)
#define Status_CALLBACK NULL
#define Status_DEFAULT NULL

extern const pb_msgdesc_t Command_msg;
extern const pb_msgdesc_t Status_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Command_fields &Command_msg
#define Status_fields &Status_msg

/* Maximum encoded size of messages (where known) */
#define Command_size                             15
#define Status_size                              15

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
