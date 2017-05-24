#include "asf.h"
#include <string.h>
#include "main.h"
#include "common/include/nm_common.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"

/************************************************************************/
/*  Defines                                                             */
/************************************************************************/

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 TCP server example --"STRING_EOL \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/**
 * LEDs
 */
#define LED_PIO_ID			ID_PIOC
#define LED_PIO       	PIOC
#define LED_PIN					8
#define LED_PIN_MASK  	(1<<LED_PIN)

// LED 1
#define LED1_PIO_ID    ID_PIOC
#define LED1_PIO       PIOC
#define LED1_PIN       19
#define LED1_PIN_MASK  (1 << LED1_PIN)

// LED 2
#define LED2_PIO_ID    ID_PIOD
#define LED2_PIO       PIOD
#define LED2_PIN       26
#define LED2_PIN_MASK  (1 << LED2_PIN)

// LED 3
#define LED3_PIO_ID    ID_PIOD
#define LED3_PIO       PIOD
#define LED3_PIN       11
#define LED3_PIN_MASK  (1 << LED3_PIN)

//Board button
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN					11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

// Botão 1
#define BUT1_PIO_ID   ID_PIOA
#define BUT1_PIO      PIOA
#define BUT1_PIN      2
#define BUT1_PIN_MASK (1 << BUT1_PIN)

// Botão 2
#define BUT2_PIO_ID   ID_PIOD
#define BUT2_PIO      PIOD
#define BUT2_PIN      30
#define BUT2_PIN_MASK (1 << BUT2_PIN)

// Botão 3
#define BUT3_PIO_ID   ID_PIOC
#define BUT3_PIO      PIOC
#define BUT3_PIN      13
#define BUT3_PIN_MASK (1 << BUT3_PIN)
/************************************************************************/
/*  Global vars                                                         */
/************************************************************************/

/** Message format definitions. */
typedef struct s_msg_wifi_product {
	uint8_t name[9];
} t_msg_wifi_product;

/** Message format declarations. */
static t_msg_wifi_product msg_wifi_product = {
	.name = MAIN_WIFI_M2M_PRODUCT_NAME,
};

/** Receive buffer definition. */
static uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];

/** Socket for TCP communication */
static SOCKET tcp_client_socket = -1;

/** Wi-Fi connection state */
static uint8_t wifi_connected;

/** Receive buffer definition. */
static uint8_t gau8ReceivedBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};
	
static uint8_t gau8SentBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};

struct sockaddr_in addr;
tstrWifiInitParam param;
int8_t ret;
int ready_to_send;
uint16_t rtn;

/************************************************************************/
/*  SOCKET MSGs                                                         */
/************************************************************************/

/** Comandos recebidos via socket */
const uint8_t MSG_SOCKET_LED_ON[]       = "Turn Led ON ! \n";
const uint8_t MSG_SOCKET_LED_ON_ACK[]   = "Led Powered ON ! \n";
const uint8_t MSG_SOCKET_LED_OFF[]      = "Turn Led OFF ! \n";
const uint8_t MSG_SOCKET_LED_OFF_ACK[]  = "Led Powered OFF \n";
const uint8_t MSG_SOCKET_LED_STATUS[]   = "Led Status ? \n";
const uint8_t MSG_SOCKET_ERRO[]         = "Command not defined \n";
enum MSG_SOCKET_COMMANDS {COMMAND_LED_ON, COMMAND_LED_OFF, COMMAND_LED_STATUS, COMMAND_ERRO};

/************************************************************************/
/*  Funcoes                                                             */
/************************************************************************/

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * Faz a interpreta��o da mensagem
 * enviada via socket txp/ip
 * retorna um numero equivalente ao
 * comando a ser executado
 */
uint8_t message_parsing(uint8_t *message){

  if(!strcmp(message, MSG_SOCKET_LED_ON)){
    printf(MSG_SOCKET_LED_ON_ACK);
    return(COMMAND_LED_ON);
  }
  else if(!strcmp(message, MSG_SOCKET_LED_OFF)){
    printf(MSG_SOCKET_LED_OFF);
    return(COMMAND_LED_OFF);
  }
  else if(!strcmp(message, MSG_SOCKET_LED_STATUS)){
     printf(MSG_SOCKET_LED_STATUS);
     return(COMMAND_LED_STATUS);
  }
  else{
    printf(MSG_SOCKET_ERRO);
    return(COMMAND_ERRO);
  }
}

/**
 * @Brief Inicializa o pino do LED
 */

void TC_init( Tc *TC, uint32_t ID_TC,  uint32_t channel, uint32_t freq ){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup�c�o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, channel, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, channel, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup�c�o no TC canal 0 */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, channel, TC_IER_CPCS);


	/* Inicializa o canal 0 do TC */
	tc_start(TC, channel);
}

/************************************************************************/
/*  Handlers                                                           */
/************************************************************************/
void TC0_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	if(ready_to_send){
	  memset(gau8SentBuffer, 0, sizeof(gau8SentBuffer));
		memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
	  sprintf((char *)gau8SentBuffer, "%s%s",MAIN_PREFIX_BUFFER,MAIN_POST);
		rtn = send(tcp_client_socket, gau8SentBuffer, strlen((char *)gau8SentBuffer), 0);
	  recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
		printf("Pronto para enviar: %d \n", rtn);
	}
}
/************************************************************************/
/*  CallBacks                                                           */
/************************************************************************/

void but_Handler(uint32_t id, uint32_t mask) {
	printf("oi, sou o handler \n");
	//limpa interrupcao do PIO
	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT_PIO);
	
	uint16_t rtn_err;
	memset(gau8SentBuffer, 0, sizeof(gau8SentBuffer));
	sprintf((char *)gau8SentBuffer, "%s%s%s", MAIN_PREFIX_BUFFER_POST, "0", MAIN_POST);
	rtn_err = send(tcp_client_socket, gau8SentBuffer, strlen((char *)gau8SentBuffer), 0);
	
	memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);	
	recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);

	printf("rtn_err: %d \n", rtn_err);
	memset(gau8SentBuffer, 0, sizeof(gau8SentBuffer));
}

void but1_Handler(uint32_t id, uint32_t mask) {
	printf("oi, sou o handler \n");
	//limpa interrupcao do PIO
	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT1_PIO);
	
	uint16_t rtn_err;
	memset(gau8SentBuffer, 0, sizeof(gau8SentBuffer));
	sprintf((char *)gau8SentBuffer, "%s%s%s", MAIN_PREFIX_BUFFER_POST, "1", MAIN_POST);
	rtn_err = send(tcp_client_socket, gau8SentBuffer, strlen((char *)gau8SentBuffer), 0);
	
	memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
	recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);

	printf("rtn_err: %d \n", rtn_err);
	memset(gau8SentBuffer, 0, sizeof(gau8SentBuffer));
}


void but2_Handler(uint32_t id, uint32_t mask) {
	printf("oi, sou o handler \n");
	//limpa interrupcao do PIO
	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT2_PIO);
	
	uint16_t rtn_err;
	memset(gau8SentBuffer, 0, sizeof(gau8SentBuffer));
	sprintf((char *)gau8SentBuffer, "%s%s%s", MAIN_PREFIX_BUFFER_POST, "2", MAIN_POST);
	rtn_err = send(tcp_client_socket, gau8SentBuffer, strlen((char *)gau8SentBuffer), 0);
	
	memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
	recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);

	printf("rtn_err: %d \n", rtn_err);
	memset(gau8SentBuffer, 0, sizeof(gau8SentBuffer));
}


void but3_Handler(uint32_t id, uint32_t mask) {
	printf("oi, sou o handler \n");
	//limpa interrupcao do PIO
	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT3_PIO);
	
	uint16_t rtn_err;
	memset(gau8SentBuffer, 0, sizeof(gau8SentBuffer));
	sprintf((char *)gau8SentBuffer, "%s%s%s", MAIN_PREFIX_BUFFER_POST, "3", MAIN_POST);
	rtn_err = send(tcp_client_socket, gau8SentBuffer, strlen((char *)gau8SentBuffer), 0);
	
	memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
	recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);

	printf("rtn_err: %d \n", rtn_err);
	memset(gau8SentBuffer, 0, sizeof(gau8SentBuffer));
}

void but_init(Pio *p_but_pio, const u_int32_t pio_id, const u_int32_t but_pin_mask) {
	printf("oi, sou o init \n");
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(pio_id);
	pio_set_input(p_but_pio, but_pin_mask, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	pio_enable_interrupt(p_but_pio, but_pin_mask);
	
	switch (but_pin_mask) {
		case BUT_PIN_MASK:
		pio_handler_set(p_but_pio, pio_id, but_pin_mask, PIO_IT_FALL_EDGE, but_Handler);
		break;
		case BUT1_PIN_MASK:
		pio_handler_set(p_but_pio, pio_id, but_pin_mask, PIO_IT_FALL_EDGE, but1_Handler);
		break;
		case BUT2_PIN_MASK:
		pio_handler_set(p_but_pio, pio_id, but_pin_mask, PIO_IT_FALL_EDGE, but2_Handler);
		break;
		case BUT3_PIN_MASK:
		pio_handler_set(p_but_pio, pio_id, but_pin_mask, PIO_IT_FALL_EDGE, but3_Handler);
		break;
	}
	
	NVIC_EnableIRQ(pio_id);
	NVIC_SetPriority(pio_id, 1);
	
}







/**
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	switch (u8Msg) {

  /* Socket connected */
  case SOCKET_MSG_CONNECT:
  {
    //memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
    //sprintf((char *)gau8ReceivedBuffer, "%s%s",MAIN_PREFIX_BUFFER,MAIN_POST);

    tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
    if (pstrConnect && pstrConnect->s8Error >= 0) {
		ready_to_send = 1;
		//printf("socket_cb: connect success!\r\n");
		//rtn = send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);
		//memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
      //recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
      } else {
		  printf("socket_cb: connect error!\r\n");
		  ready_to_send = 0;
		  close(tcp_client_socket);
		  tcp_client_socket = -1;
    }
  }
  break;

	/* Message send */
	case SOCKET_MSG_SEND:
	{
		//printf("socket_cb: send success!\r\n");
		//recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
	  //printf("TCP Server Test Complete!\r\n");
		//printf("close socket\n");
		//close(tcp_client_socket);
		//close(tcp_server_socket);
	}
	break;

	/* Message receive */
	case SOCKET_MSG_RECV:
	{
		tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
    uint8_t  messageAck[64];
    uint16_t messageAckSize;
    uint8_t  command;

		if (pstrRecv && pstrRecv->s16BufferSize > 0) {

			// Para debug das mensagens do socket
			printf("%s",pstrRecv->pu8Buffer);
			const char *last = &pstrRecv->pu8Buffer[pstrRecv->s16BufferSize-4];
			printf("%s", last);
			if(last[0] == '1')
			pio_clear(LED_PIO, LED_PIN_MASK);
			else if(last[0] == '0')
			pio_set(LED_PIO, LED_PIN_MASK);
			if(last[1] == '1')
			pio_clear(LED1_PIO, LED1_PIN_MASK);
			else if(last[1] == '0')
			pio_set(LED1_PIO, LED1_PIN_MASK);
			if(last[2] == '1')
			pio_clear(LED2_PIO, LED2_PIN_MASK);
			else if(last[2] == '0')
			pio_set(LED2_PIO, LED2_PIN_MASK);
			if(last[3] == '1')
			pio_clear(LED3_PIO, LED3_PIN_MASK);
			else if(last[3] == '0')
			pio_set(LED3_PIO, LED3_PIN_MASK);

      // limpa o buffer de recepcao e tx
      memset(pstrRecv->pu8Buffer, 0, pstrRecv->s16BufferSize);

      // envia a resposta
      //delay_s(1);
      //sprintf((char *)gau8ReceivedBuffer, "%s%s",MAIN_PREFIX_BUFFER,MAIN_POST);
      //send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);

      // Requista novos dados
      //recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);

 		} else {
			printf("socket_cb: recv error!\r\n");
			close(tcp_client_socket);
			tcp_client_socket = -1;
		}
	}

	break;

	default:
		break;
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			wifi_connected = 0;
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}
	}
	break;

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		wifi_connected = 1;
		printf("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
	}
	break;

	default:
		break;
	}
}

/************************************************************************/
/*  Main                                                               */
/************************************************************************/

/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then test function of TCP server.
 *
 * \return program return value.
 */
int main(void)
{
	ready_to_send = 0;
	/* Initialize the board. */
	sysclk_init();
	board_init();

	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);

	/* Initialize the BSP. */
	nm_bsp_init();

	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIN_MASK, 0, 0, 0);

	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIN_MASK, 0, 0, 0);
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIN_MASK, 0, 0, 0);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIN_MASK, 0, 0, 0);
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIN_MASK, 0, 0, 0);

	but_init(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK);
	but_init(BUT1_PIO, BUT1_PIO_ID, BUT1_PIN_MASK);
	but_init(BUT2_PIO, BUT2_PIO_ID, BUT2_PIN_MASK);
	but_init(BUT3_PIO, BUT3_PIO_ID, BUT3_PIN_MASK);
	/* Initialize socket address structure. */
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(MAIN_SERVER_PORT);
	addr.sin_addr.s_addr = MAIN_SERVER_IP;

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}
	/* Initialize GET TC */
	TC_init(TC0, ID_TC0, 0, 10);

	/* Initialize socket module */
	socketInit();
	registerSocketCallback(socket_cb, NULL);

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
	while (1) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);

		if (wifi_connected == M2M_WIFI_CONNECTED) {
			if (tcp_client_socket < 0) {
				if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
					printf("main: failed to create TCP client socket error!\r\n");
				}

				/* Connect TCP client socket. */
				if (connect(tcp_client_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR ) {
					printf("main: failed to connect socket error!\r\n");
					close(tcp_client_socket);
					}else{
					printf("Conectado ! \n");
				}
			}
		}
	}

	return 0;
}
