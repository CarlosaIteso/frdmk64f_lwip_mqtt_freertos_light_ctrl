/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lwip/opt.h"

#if LWIP_IPV4 && LWIP_RAW && LWIP_NETCONN && LWIP_DHCP && LWIP_DNS

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_phy.h"

#include "lwip/api.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dhcp.h"
#include "lwip/netdb.h"
#include "lwip/netifapi.h"
#include "lwip/prot/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "lwip_mqtt_id.h"

#include "ctype.h"
#include "stdio.h"

#include "fsl_phyksz8081.h"
#include "fsl_enet_mdio.h"
#include "fsl_device_registers.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

//carlosa: command definition
#define command_off 0
#define command_on 1

//carlosa: defining light control functionality
#define function_shutoff_time 0
#define function_light_1 1
#define function_light_2 2

//carlosa: setting lights GPIO and PIN
#define light_1_GPIO     BOARD_LED_RED_GPIO
#define light_1_GPIO_PIN BOARD_LED_RED_PIN

#define light_2_GPIO     BOARD_LED_BLUE_GPIO
#define light_2_GPIO_PIN BOARD_LED_BLUE_PIN

//carlosa: defining sub topics
#define light_1_command_topic "light_ctrl/light_1/command"
#define light_2_command_topic "light_ctrl/light_2/command"
#define shut_off_command_topic "light_ctrl/shut_off/time"

//carlosa: defining pub topics
#define light_1_state_topic "light_ctrl/light_1/state"
#define light_2_state_topic "light_ctrl/light_2/state"

/* @TEST_ANCHOR */

//carlosa: MAC address changed due to DHCP issues
/* MAC address configuration. */
#ifndef configMAC_ADDR
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x06 \
    }
#endif

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_ops

/* PHY operations. */
#define EXAMPLE_PHY_OPS phyksz8081_ops

/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)

/* GPIO pin configuration. */
#define BOARD_LED_GPIO       BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN   BOARD_LED_RED_GPIO_PIN
#define BOARD_SW_GPIO        BOARD_SW3_GPIO
#define BOARD_SW_GPIO_PIN    BOARD_SW3_GPIO_PIN
#define BOARD_SW_PORT        BOARD_SW3_PORT
#define BOARD_SW_IRQ         BOARD_SW3_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_SW3_IRQ_HANDLER


#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

//to-do: change to actual broker address.
/*! @brief MQTT server host name or IP address. */
#define EXAMPLE_MQTT_SERVER_HOST "broker.hivemq.com"

//to-do: change to actual broker port
/*! @brief MQTT server port number. */
#define EXAMPLE_MQTT_SERVER_PORT 1883

/*! @brief Stack size of the temporary lwIP initialization thread. */
#define INIT_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary lwIP initialization thread. */
#define INIT_THREAD_PRIO DEFAULT_THREAD_PRIO

/*! @brief Stack size of the temporary initialization thread. */
#define APP_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary initialization thread. */
#define APP_THREAD_PRIO DEFAULT_THREAD_PRIO

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void connect_to_mqtt(void *ctx);

static void light_ctrl_main();

static void set_light_command();

static void set_light_state();

/*******************************************************************************
 * Variables
 ******************************************************************************/

//carlosa: global variable for topic index
static uint8_t topic_index;

//carlosa: global variable for light command
static uint8_t light_command;

//carlosa: global variable for light state
char light_1_state = '0';
char light_2_state = '0';

static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t phyHandle   = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};

/*! @brief MQTT client data. */
static mqtt_client_t *mqtt_client;

/*! @brief MQTT client ID string. */
static char client_id[40];

/*! @brief MQTT client information. */
static const struct mqtt_connect_client_info_t mqtt_client_info = {
    .client_id   = (const char *)&client_id[0],
    .client_user = "carlosa_o2023", //to-do: change user and password
    .client_pass = "o2023",
    .keep_alive  = 100,
	//carlosa: changed will config
    .will_topic  = "light_ctrl/k64_status",
    .will_msg    = "k64f is not connected",
    .will_qos    = 1,
    .will_retain = 1,
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    .tls_config = NULL,
#endif
};

/*! @brief MQTT broker IP address. */
static ip_addr_t mqtt_addr;

/*! @brief Indicates connection to MQTT broker. */
static volatile bool connected = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Called when subscription request finishes.
 */
static void mqtt_topic_subscribed_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Subscribed to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to subscribe to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Called when there is a message on a subscribed topic.
 */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    LWIP_UNUSED_ARG(arg);

    PRINTF("Received %u bytes from the topic \"%s\" \n", tot_len, topic);

    //carlosa: update light_index according to the topic of the message recieved
    if (strcmp(topic, light_1_command_topic) == 0) {
        topic_index = 1;
    } else if (strcmp(topic, light_2_command_topic) == 0) {
        topic_index = 2;
    } else if (strcmp(topic, shut_off_command_topic) == 0) {
        topic_index = 0;
    } else {
        PRINTF("Unknown topic.\n");
    }
}

/*!
 * @brief Called when recieved incoming published message fragment.
 */
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    LWIP_UNUSED_ARG(arg);

    //carlosa: check if data is more than 1 byte
	if (len > 1)
	{
		PRINTF("Incomming message must be 1 byte len. light command is Off for 0 and On for 1");
		return;
	}

	//carlosa: updating the light command
	light_command = data[0] - '0';

	if (isprint(data[0]))
	{
	    PRINTF("Light command: %c\n", (char)data[0]); // Print as character if it's printable
	}
	else
	{
	    PRINTF("Light command: \\x%02x\n", data[0]); // Print as hexadecimal if it's not printable
	}

	//carlosa: call light_ctrl_main
	light_ctrl_main();
}

/*!
 * @brief Subscribe to MQTT topics.
 */
static void mqtt_subscribe_topics(mqtt_client_t *client){

	//carlosa: added topics for commands of light_1 and light_2
    static const char *topics[] = {light_1_command_topic, light_2_command_topic, shut_off_command_topic};
    //carlosa: changed of qos
    int qos[]                   = {1, 1, 1};
    err_t err;
    int i;

    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb,
                            LWIP_CONST_CAST(void *, &mqtt_client_info));

    for (i = 0; i < ARRAY_SIZE(topics); i++)
    {
        err = mqtt_subscribe(client, topics[i], qos[i], mqtt_topic_subscribed_cb, LWIP_CONST_CAST(void *, topics[i]));

        if (err == ERR_OK)
        {
            PRINTF("Subscribing to the topic \"%s\" with QoS %d...\r\n", topics[i], qos[i]);
        }
        else
        {
            PRINTF("Failed to subscribe to the topic \"%s\" with QoS %d: %d.\r\n", topics[i], qos[i], err);
        }
    }
}

/*!
 * @brief Called when connection state changes.
 */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    const struct mqtt_connect_client_info_t *client_info = (const struct mqtt_connect_client_info_t *)arg;

    connected = (status == MQTT_CONNECT_ACCEPTED);

    switch (status)
    {
        case MQTT_CONNECT_ACCEPTED:
            PRINTF("MQTT client \"%s\" connected.\r\n", client_info->client_id);
            mqtt_subscribe_topics(client);
            break;

        case MQTT_CONNECT_DISCONNECTED:
            PRINTF("MQTT client \"%s\" not connected.\r\n", client_info->client_id);
            /* Try to reconnect 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_TIMEOUT:
            PRINTF("MQTT client \"%s\" connection timeout.\r\n", client_info->client_id);
            /* Try again 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_REFUSED_PROTOCOL_VERSION:
        case MQTT_CONNECT_REFUSED_IDENTIFIER:
        case MQTT_CONNECT_REFUSED_SERVER:
        case MQTT_CONNECT_REFUSED_USERNAME_PASS:
        case MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_:
            PRINTF("MQTT client \"%s\" connection refused: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;

        default:
            PRINTF("MQTT client \"%s\" connection status: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;
    }
}

/*!
 * @brief Starts connecting to MQTT broker. To be called on tcpip_thread.
 */
static void connect_to_mqtt(void *ctx)
{
    LWIP_UNUSED_ARG(ctx);

    PRINTF("Connecting to MQTT broker at %s...\r\n", ipaddr_ntoa(&mqtt_addr));

    mqtt_client_connect(mqtt_client, &mqtt_addr, EXAMPLE_MQTT_SERVER_PORT, mqtt_connection_cb,
                        LWIP_CONST_CAST(void *, &mqtt_client_info), &mqtt_client_info);
}

/*!
 * @brief Called when publish request finishes.
 */
static void mqtt_message_published_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Published to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to publish to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Publishes a message. To be called on tcpip_thread.
 */
static void publish_message(void *ctx)
{
    static const char *topic[]   = {light_1_state_topic, light_2_state_topic};

    LWIP_UNUSED_ARG(ctx);

    //carlosa: publish lights state
    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic[0]);
    mqtt_publish(mqtt_client, topic[0], &light_1_state, sizeof(light_1_state), 1, 1, mqtt_message_published_cb, (void *)topic[0]);
    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic[1]);
    mqtt_publish(mqtt_client, topic[1], &light_2_state, sizeof(light_2_state), 1, 1, mqtt_message_published_cb, (void *)topic[1]);
}

static void publish_message_connected(void *ctx)
{
	static const char *topic   = "light_ctrl/k64_status";
	static const char *message = "k64f is connected";
    LWIP_UNUSED_ARG(ctx);

    //carlosa: publish that k64f is connected to light_ctrl/k64_status
    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);
    mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 1, mqtt_message_published_cb, (void *)topic);
}

/*!
 * @brief Application thread.
 */
static void app_thread(void *arg)
{
    struct netif *netif = (struct netif *)arg;
    struct dhcp *dhcp;
    err_t err;
    int i;

    /* Wait for address from DHCP */
    PRINTF("Getting IP address from DHCP...\r\n");

    do
    {
        if (netif_is_up(netif))
        {
            dhcp = netif_dhcp_data(netif);
        }
        else
        {
            dhcp = NULL;
        }

        sys_msleep(20U);

    } while ((dhcp == NULL) || (dhcp->state != DHCP_STATE_BOUND));

    PRINTF("\r\nIPv4 Address     : %s\r\n", ipaddr_ntoa(&netif->ip_addr));
    PRINTF("IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&netif->netmask));
    PRINTF("IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa(&netif->gw));

    /*
     * Check if we have an IP address or host name string configured.
     * Could just call netconn_gethostbyname() on both IP address or host name,
     * but we want to print some info if goint to resolve it.
     */
    if (ipaddr_aton(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr) && IP_IS_V4(&mqtt_addr))
    {
        /* Already an IP address */
        err = ERR_OK;
    }
    else
    {
        /* Resolve MQTT broker's host name to an IP address */
        PRINTF("Resolving \"%s\"...\r\n", EXAMPLE_MQTT_SERVER_HOST);
        err = netconn_gethostbyname(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr);
    }

    if (err == ERR_OK)
    {
        /* Start connecting to MQTT broker from tcpip_thread */
        err = tcpip_callback(connect_to_mqtt, NULL);
        if (err != ERR_OK)
        {
            PRINTF("Failed to invoke broker connection on the tcpip_thread: %d.\r\n", err);
        }
    }
    else
    {
        PRINTF("Failed to obtain IP address: %d.\r\n", err);
    }

    //carlosa: publish k64f status as connectedd
	/* Publish some messages */
	for (i = 0; i < 5;)
	{
		if (connected)
		{
			err = tcpip_callback(publish_message_connected, NULL);
			if (err != ERR_OK)
			{
				PRINTF("Failed to invoke publishing of a message on the tcpip_thread: %d.\r\n", err);
			}
			i++;
		}

		sys_msleep(1000U);
	}

	//initial state for lights
	tcpip_callback(publish_message, NULL);

    vTaskDelete(NULL);
}

static void generate_client_id(void)
{
    uint32_t mqtt_id[MQTT_ID_SIZE];
    int res;

    get_mqtt_id(&mqtt_id[0]);

    res = snprintf(client_id, sizeof(client_id), "nxp_%08lx%08lx%08lx%08lx", mqtt_id[3], mqtt_id[2], mqtt_id[1],
                   mqtt_id[0]);
    if ((res < 0) || (res >= sizeof(client_id)))
    {
        PRINTF("snprintf failed: %d\r\n", res);
        while (1)
        {
        }
    }
}

/*!
 * @brief Initializes lwIP stack.
 *
 * @param arg unused
 */
static void stack_init(void *arg)
{
    static struct netif netif;
    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
    ethernetif_config_t enet_config = {
        .phyHandle  = &phyHandle,
        .macAddress = configMAC_ADDR,
    };

    LWIP_UNUSED_ARG(arg);
    generate_client_id();

    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

    IP4_ADDR(&netif_ipaddr, 0U, 0U, 0U, 0U);
    IP4_ADDR(&netif_netmask, 0U, 0U, 0U, 0U);
    IP4_ADDR(&netif_gw, 0U, 0U, 0U, 0U);

    tcpip_init(NULL, NULL);

    LOCK_TCPIP_CORE();
    mqtt_client = mqtt_client_new();
    UNLOCK_TCPIP_CORE();
    if (mqtt_client == NULL)
    {
        PRINTF("mqtt_client_new() failed.\r\n");
        while (1)
        {
        }
    }

    netifapi_netif_add(&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN,
                       tcpip_input);
    netifapi_netif_set_default(&netif);
    netifapi_netif_set_up(&netif);

    netifapi_dhcp_start(&netif);

    PRINTF("\r\n************************************************\r\n");
    PRINTF(" MQTT client example\r\n");
    PRINTF("************************************************\r\n");

    if (sys_thread_new("app_task", app_thread, &netif, APP_THREAD_STACKSIZE, APP_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("stack_init(): Task creation failed.", 0);
    }

    vTaskDelete(NULL);
}

/*!
 * @brief Main function
 */
int main(void)
{
    SYSMPU_Type *base = SYSMPU;
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    /* Disable SYSMPU. */
    base->CESR &= ~SYSMPU_CESR_VLD_MASK;

    /* Initialize lwIP from thread */
    if (sys_thread_new("main", stack_init, NULL, INIT_THREAD_STACKSIZE, INIT_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("main(): Task creation failed.", 0);
    }

    vTaskStartScheduler();

    /* Will not get here unless a task calls vTaskEndScheduler ()*/
    return 0;
}

/*******************************************************************************
 * light_ctrl application
 ******************************************************************************/

static void light_ctrl_main(){
	// to-do: light_ctrl_main implementation

	//carlosa: call set_light_command
	set_light_command();

	//carlosa: call set_light_state
	set_light_state();
}

static void set_light_command(){

	//carlosa: turn off/on light 1
	if( topic_index == function_light_1){

		PRINTF("Setting command for light 1.\n");
		if (light_command == command_off){
			GPIO_PortClear(light_1_GPIO, 1u << light_1_GPIO_PIN);

			//carlosa: update light 1 state
			light_1_state = '0';
		}
		else if (light_command == command_on){
			GPIO_PortSet(light_1_GPIO, 1u << light_1_GPIO_PIN);

			//carlosa: update light 1 state
			light_1_state = '1';
		}
		else{
			PRINTF("Unkown command.\n");
		}
	}

	//carlosa: turn off/on light 2
	else if (topic_index == function_light_2){

		PRINTF("Setting command for light 2.\n");
		if (light_command == command_off){
			GPIO_PortClear(light_2_GPIO, 1u << light_2_GPIO_PIN);

			//carlosa: update light 2 state
			light_2_state = '0';
		}
		else if (light_command == command_on){
			GPIO_PortSet(light_2_GPIO, 1u << light_2_GPIO_PIN);

			//carlosa: update light 2 state
			light_2_state = '1';
		}
		else{
			PRINTF("Unkown command.\n");
		}
	}

	//turn off all lights in light_command seconds
	else if (topic_index == function_shutoff_time){

		PRINTF("Turning of lights. \n");
		vTaskDelay(light_command*1000);
		GPIO_PortClear(light_1_GPIO, 1u << light_1_GPIO_PIN);
		GPIO_PortClear(light_2_GPIO, 1u << light_2_GPIO_PIN);

		light_1_state = '0';
		light_2_state = '0';

	}

	else{
		PRINTF("Unkown topic.\n");
	}
}

static void set_light_state(){

	//carlosa: call tcpip_callback to publish the new lights state
	tcpip_callback(publish_message, NULL);
}

#endif
