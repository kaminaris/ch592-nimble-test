#ifndef _CH5xxCFG_H
#define _CH5xxCFG_H

#ifndef CH59x
# error CH59x not defined
#else

# ifndef MYNEWT_VAL_MCU_TARGET__CH59x
#  define MYNEWT_VAL_MCU_TARGET__CH59x (1)
# endif

# ifndef MYNEWT_VAL_NIMBLE_HOST_TASK_STACK_SIZE
#  define MYNEWT_VAL_NIMBLE_HOST_TASK_STACK_SIZE (1024)
# endif

#define MYNEWT_VAL_BLE_STORE_CONFIG_PERSIST (0)
#define MYNEWT_VAL_BLE_MESH (0)
#define MYNEWT_VAL_BLE_MESH_PROXY (0)
#define MYNEWT_VAL_BLE_MESH_PROV (0)

// #define CONFIG_BT_NIMBLE_ACL_BUF_COUNT 4
// #define CONFIG_BT_NIMBLE_ACL_BUF_SIZE 255
// #define CONFIG_BT_NIMBLE_HS_FLOW_CTRL_THRESH 2
// #define CONFIG_BT_NIMBLE_HS_FLOW_CTRL_ITVL 10

#define MYNEWT_VAL_MSYS_1_BLOCK_COUNT (6)
#define MYNEWT_VAL_BLE_TRANSPORT_ACL_COUNT (4)  // Down from 10
#define MYNEWT_VAL_BLE_TRANSPORT_EVT_COUNT (8)  // Down from 30
#define MYNEWT_VAL_BLE_MAX_CONNECTIONS (1)  // Down from 3

// #define MYNEWT_VAL_BLE_ROLE_CENTRAL (0)        // Disable central role
// #define MYNEWT_VAL_BLE_ROLE_OBSERVER (0)       // Disable observer role
// #define MYNEWT_VAL_BLE_ROLE_BROADCASTER (0)    // Optional, if not needed


#endif // NRF52840_XXAA
#endif // _NRF52840CFG_H
