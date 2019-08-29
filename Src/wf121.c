/*
 * wf121.c
 *
 *  Created on: Jun 10, 2019
 *      Author: burr
 */

#include "wifi_bglib.h"
#include "main.h"
#include "wf121.h"
//#define AP_MODE // with TCPIP Server

#define TCP_SERVER //use this if connect to WIFI

// 3 Below not stable for operation
//#define UDP_CONNECT
//#define UDP_SERVER // doesn't work
//#define TCP_CONNECT

//#define CHECKID


ipv4 ip_address = {.a[0] = 192, .a[1] = 168, .a[2] = 1, .a[3] = 222}; //own IP

#define PORT 5707
ipv4 netmask = {.a[0] = 255, .a[1] = 255, .a[2] = 255, .a[3] = 0};
ipv4 gateway = {.a[0] = 192, .a[1] = 168, .a[2] = 1, .a[3] = 1};
ipv4 dns = {.a[0] = 8, .a[1] = 8, .a[2] = 8, .a[3] = 8};


#ifdef TCP_CONNECT
ipv4 destip_address = {.a[0] = 192, .a[1] = 168, .a[2] = 1, .a[3] = 148};
#endif

#ifdef UDP_CONNECT
ipv4 udp_address = {.a[0] = 192, .a[1] = 168, .a[2] = 1, .a[3] = 148}; //to connect to
#endif


void uart_output(uint8 len1,uint8* data1,uint16 len2,uint8* data2){
	// implementation of the output function using HAL blocking functions
	HAL_UART_Transmit(&huart1, data1, len1,100);
	HAL_UART_Transmit(&huart1, data2, len2,100);
}

void wifi_init(uint8_t* rxbuffer){
	BGLIB_INITIALIZE(uart_output);

	HAL_GPIO_WritePin(MCLR_WF_GPIO_Port, MCLR_WF_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(MCLR_WF_GPIO_Port, MCLR_WF_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_UART_Receive(&huart1, rxbuffer, BGLIB_MSG_MAXLEN, 1000); //To make sure nothing is in UART Buffer

	for (uint16_t i=0; i<BGLIB_MSG_MAXLEN-1; i++){
		rxbuffer[i]=0;}

	rxcplt=0;
	msg_length=0;

    // in interrupt 4 bytes(header) is written and checked for message length, if msg_len>0 the rest of the message is read with a blocking function
	// here the UART ISR is enabled
	HAL_UART_Receive_IT(&huart1, &rxbuffer[0], BGLIB_MSG_HEADER_LEN);

	wifi_cmd_system_reset(0);
}

void wifi_connect(uint8_t *rxbuffer, uint8_t *streamingset){

#ifdef AP_MODE
	static uint8_t apSSID[] = {'n', 'i', 'l', 'm', 'a', 'p'};
#else
	static uint8_t SSID[] = {'F','A','S','T','W','E','B','-','1','-','D','0','9','1','9','9'};
	static uint8_t pw[] = {'6','A','D','8','B','8','5','5','B','8'};


#endif
	struct wifi_cmd_packet* pck;
	pck=BGLIB_MSG(rxbuffer);
	switch(BGLIB_MSG_ID(rxbuffer))
	{
	// Switchcase Message IDs are shown in the order as the command responses and events should be received from the module
	case wifi_evt_system_boot_id:
		// Module has booted, synchronize handled notifications to received interrupts
		printf("System has booted\n");
		connected = 0;
		printf("Sending command: wifi_cmd_sme_set_operating_mode(2)\n");
#ifdef AP_MODE
		wifi_cmd_sme_set_operating_mode(2);
#else
		wifi_cmd_sme_set_operating_mode(1); //1 client mode, 2 ap mode
#endif
		break;

	case wifi_rsp_sme_set_operating_mode_id:
		printf("Command response received: wifi_rsp_sme_set_operating_mode_id\n");
		wifi_cmd_system_set_max_power_saving_state(0);
		break;

	case wifi_rsp_system_set_max_power_saving_state_id:
		printf("Sending command: wifi_cmd_sme_wifi_on\n");
		wifi_cmd_sme_wifi_on();
		break;

	case wifi_rsp_sme_wifi_on_id:
		printf("Command response received: wifi_rsp_sme_wifi_on_id\n");
		break;

	case wifi_evt_sme_wifi_is_on_id:
		printf("Event received: wifi_evt_sme_wifi_is_on\n");
		printf("Sending command: wifi_cmd_tcpip_configure\n");
#ifdef AP_MODE
		//wifi ap is not secured
		wifi_cmd_tcpip_dns_configure(0,dns.u);
#else
		wifi_cmd_sme_set_password(sizeof(pw), pw);
#endif
		break;

	case wifi_rsp_sme_set_password_id:
		printf("Command response received: wifi_rsp_sme_set_password_id\n");
		wifi_cmd_tcpip_dns_configure(0,dns.u);
		break;

	case wifi_evt_tcpip_dns_configuration_id:
		if (!connected)
			wifi_cmd_tcpip_configure(ip_address.u, netmask.u, gateway.u, 0); //0 = use static ip adress
		break;

	case wifi_rsp_tcpip_configure_id:
		printf("Command response received: wifi_rsp_tcpip_configure_id\n");
		break;

	case wifi_evt_tcpip_configuration_id:
		/* This event is triggered by other commands such as start_ap_mode command.
		 * The ap_mode_started bool prevents it from being issued more than once. */
		printf("Event received: wifi_evt_tcpip_configuration_id\n");
		if(!connected) {
			printf("Sending command: wifi_cmd_sme_connect_ssid\n");
#ifdef AP_MODE
			wifi_cmd_sme_start_ap_mode(11,0,sizeof(apSSID),apSSID);

#else
			wifi_cmd_sme_connect_ssid(sizeof(SSID), SSID);
#endif
		}
		break;

	case wifi_rsp_sme_connect_ssid_id:
		printf("Command response received: wifi_rsp_sme_connect_ssid_id\n");
		break;

#ifdef AP_MODE
	case wifi_evt_sme_ap_mode_started_id:
		printf("Event received: wifi_evt_sme_ap_mode_started_id\n");
		printf("Sending command: wifi_cmd_https_enable\n");
		connected = 1;
		wifi_cmd_https_enable(0, 1, 0);
		break;
	case wifi_rsp_https_enable_id:
		//only needed for ap mode
		printf("Command response received: wifi_rsp_https_enable_id\n");
		printf("Sending command: wifi_cmd_tcpip_start_tcp_server\n");
		wifi_cmd_tcpip_start_tcp_server(PORT,-1);
		break;
#endif

	case wifi_evt_sme_connected_id:
		printf("connected to micrel network");
		connected=1;
#ifdef UDP_CONNECT
		wifi_cmd_tcpip_udp_connect(udp_address.u, PORT, -1);
#endif
#ifdef UDP_SERVER
		wifi_cmd_tcpip_start_udp_server(PORT, -1)
#endif
#ifdef TCP_SERVER
		wifi_cmd_tcpip_start_tcp_server(PORT, 31);  //data received will be dropped
#endif
#ifdef TCP_CONNECT
		wifi_cmd_tcpip_tcp_connect(destip_address.u, PORT, 31); //data received will be dropped
#endif
		break;

	case wifi_rsp_tcpip_tcp_connect_id:
		printf("Command response received: wifi_rsp_tcpip_tcp_connect_id\n");
		break;

	case wifi_rsp_tcpip_udp_connect_id:
		saved_endpoint=pck->rsp_tcpip_udp_connect.endpoint;
		wifi_cmd_endpoint_set_streaming_destination(1 ,saved_endpoint);//set destination saved_endpoint from UART2
		printf("Command response received: wifi_rsp_tcpip_udp_connect_id\n");
		break;


	case wifi_rsp_tcpip_start_tcp_server_id:
		printf("Command response received: wifi_rsp_tcpip_start_tcp_server_id\n");
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);//TCPIP Server started, ready to connect to now
		break;

	case wifi_rsp_tcpip_start_udp_server_id:
		saved_endpoint=pck->rsp_tcpip_start_udp_server.endpoint;
		wifi_cmd_endpoint_set_streaming_destination(1 ,saved_endpoint);//set destination to saved_endpoint from UART2
		printf("Command response received: wifi_rsp_tcpip_start_udp_server_id\n");
		break;

	case wifi_evt_tcpip_endpoint_status_id:
		if(pck->evt_tcpip_endpoint_status.local_ip.u != 0)
		{
			printf("Session opened with IP %03d.%03d.%03d.%03d on endpoint %d\n",
					pck->evt_tcpip_endpoint_status.remote_ip.a[0],
					pck->evt_tcpip_endpoint_status.remote_ip.a[1],
					pck->evt_tcpip_endpoint_status.remote_ip.a[2],
					pck->evt_tcpip_endpoint_status.remote_ip.a[3],
					pck->evt_tcpip_endpoint_status.endpoint
			);
			//wifi_cmd_endpoint_send(pck->evt_tcpip_endpoint_status.endpoint, sizeof(tcp_message), tcp_message); //This can be used to send single messages without streaming
			saved_endpoint=pck->evt_tcpip_endpoint_status.endpoint;
			wifi_cmd_endpoint_set_streaming_destination(1 ,saved_endpoint);//set destination saved_endpoint from UART2

			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
		}
		break;

	case wifi_rsp_endpoint_set_streaming_destination_id:
		printf("streaming destination set");
		wifi_cmd_endpoint_set_streaming(1, 1);//uart2 as endpoint, to another endpoint
		*streamingset=1; //This is a shortcut, since several other messages are received after. These get prevent by turning of the UART Receive Function in main.c. The more elegant way would be to check for PCK content of responses

#ifdef TCP_CONNECT
		*streamingset=1;
#endif
#ifdef UDP_CONNECT
		*streamingset=1;
#endif
#ifdef UDP_SERVER
		*streamingset=1;
#endif
		break;


	case wifi_rsp_endpoint_set_streaming_id:
		printf("streaming set");
		break;

	case wifi_evt_endpoint_closing_id:
		printf("Session on endpoint %d closed\n",pck->evt_endpoint_closing.endpoint);
		break;


#ifdef AP_MODE
	case wifi_evt_sme_ap_client_joined_id:
		printf("Client joined with MAC %02X %02X %02X %02X %02X %02X\n",
				pck->evt_sme_ap_client_joined.mac_address.addr[0],
				pck->evt_sme_ap_client_joined.mac_address.addr[1],
				pck->evt_sme_ap_client_joined.mac_address.addr[2],
				pck->evt_sme_ap_client_joined.mac_address.addr[3],
				pck->evt_sme_ap_client_joined.mac_address.addr[4],
				pck->evt_sme_ap_client_joined.mac_address.addr[5]
		);
		break;

	case wifi_evt_sme_ap_client_left_id:
		printf("Client left with MAC %02X %02X %02X %02X %02X %02X\n",
				pck->evt_sme_ap_client_joined.mac_address.addr[0],
				pck->evt_sme_ap_client_joined.mac_address.addr[1],
				pck->evt_sme_ap_client_joined.mac_address.addr[2],
				pck->evt_sme_ap_client_joined.mac_address.addr[3],
				pck->evt_sme_ap_client_joined.mac_address.addr[4],
				pck->evt_sme_ap_client_joined.mac_address.addr[5]
		);
		break;
#endif

	case wifi_evt_endpoint_data_id: //BGLIB_ID for received Data
		break;

	case wifi_evt_endpoint_status_id:
			// Last event received after set_streaming
//		if ((pck->evt_endpoint_status.active==1)&&(pck->evt_endpoint_status.destination==3)){
//			wifi_cmd_endpoint_set_active(1, 0); //disables uart endpoint, no responses/events go out of the wf121 module anymore
//		}
		break;

	default:
		break;
	}

// erase buffer header, receive next package
	for (uint16_t i=0; i<BGLIB_MSG_HEADER_LEN; i++){
		rxbuffer[i]=0;}
	rxcplt=0;
	msg_length=0;

	HAL_UART_Receive_IT(&huart1, &rxbuffer[0], BGLIB_MSG_HEADER_LEN);

}

void wifi_check(uint8_t* rxbuffer, uint8_t *streamingset){

	//All possible WiFi Events
#ifdef CHECKID
	switch(BGLIB_MSG_ID(rxbuffer)){
	case wifi_evt_dfu_boot_id:
		HAL_Delay(1);
		break;
	case wifi_evt_system_boot_id:
		HAL_Delay(1);
		break;
	case wifi_evt_system_state_id:
		HAL_Delay(1);
		break;
	case wifi_evt_system_sw_exception_id:
		HAL_Delay(1);
		break;
	case wifi_evt_system_power_saving_state_id:
		HAL_Delay(1);
		break;
	case wifi_evt_config_mac_address_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_wifi_is_on_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_wifi_is_off_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_scan_result_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_scan_result_drop_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_scanned_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_connected_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_disconnected_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_interface_status_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_connect_failed_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_connect_retry_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_ap_mode_started_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_ap_mode_stopped_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_ap_mode_failed_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_ap_client_joined_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_ap_client_left_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_scan_sort_result_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_scan_sort_finished_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_wps_stopped_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_wps_completed_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_wps_failed_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_wps_credential_ssid_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_wps_credential_password_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sme_signal_quality_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_configuration_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_dns_configuration_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_endpoint_status_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_dns_gethostbyname_result_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_udp_data_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_mdns_started_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_mdns_failed_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_mdns_stopped_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_dnssd_service_started_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_dnssd_service_failed_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_dnssd_service_stopped_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_dhcp_configuration_id:
		HAL_Delay(1);
		break;
	case wifi_evt_tcpip_dhcp_client_id:
		HAL_Delay(1);
		break;
	case wifi_evt_endpoint_syntax_error_id:
		HAL_Delay(1);
		break;
	case wifi_evt_endpoint_data_id:
		HAL_Delay(1);
		break;
	case wifi_evt_endpoint_status_id:
		HAL_Delay(1);
		break;
	case wifi_evt_endpoint_closing_id:
		HAL_Delay(1);
		break;
	case wifi_evt_endpoint_error_id:
		HAL_Delay(1);
		break;
	case wifi_evt_hardware_soft_timer_id:
		HAL_Delay(1);
		break;
	case wifi_evt_hardware_change_notification_id:
		HAL_Delay(1);
		break;
	case wifi_evt_hardware_external_interrupt_id:
		HAL_Delay(1);
		break;
	case wifi_evt_hardware_rtc_alarm_id:
		HAL_Delay(1);
		break;
	case wifi_evt_hardware_uart_conf_id:
		HAL_Delay(1);
		break;
	case wifi_evt_flash_ps_key_id:
		HAL_Delay(1);
		break;
	case wifi_evt_flash_ps_key_changed_id:
		HAL_Delay(1);
		break;
	case wifi_evt_flash_low_voltage_id:
		HAL_Delay(1);
		break;
	case wifi_evt_https_on_req_id:
		HAL_Delay(1);
		break;
	case wifi_evt_https_button_id:
		HAL_Delay(1);
		break;
	case wifi_evt_https_api_request_id:
		HAL_Delay(1);
		break;
	case wifi_evt_https_api_request_header_id:
		HAL_Delay(1);
		break;
	case wifi_evt_https_api_request_data_id:
		HAL_Delay(1);
		break;
	case wifi_evt_https_api_request_finished_id:
		HAL_Delay(1);
		break;
	case wifi_evt_ethernet_link_status_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sdhc_ready_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sdhc_fdata_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sdhc_ffile_id:
		HAL_Delay(1);
		break;
	case wifi_evt_sdhc_fpwd_id:
		HAL_Delay(1);
		break;

	}
#endif

//	Example on how to reopen connection after drop. Not working
//	pck=BGLIB_MSG(rxbuffer);
//	if(BGLIB_MSG_ID(rxbuffer)==wifi_evt_endpoint_closing_id){
//	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1); //triggers spi read, synced to conv at Tim1
//	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2); //for triggering CONV at NISO
//	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); //for triggering CONV at ISO (current and voltage)
//	started_measurement=0;
//	streamingset=0;
//	saved_endpoint=0;
//	connected = 0;
//	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
//  wifi_init();
//	}

	// erase buffer header, receive next package
	for (uint16_t i=0; i<BGLIB_MSG_HEADER_LEN; i++){
		rxbuffer[i]=0;}
	rxcplt=0;
	msg_length=0;

	HAL_UART_Receive_IT(&huart1, &rxbuffer[0], BGLIB_MSG_HEADER_LEN);
}
