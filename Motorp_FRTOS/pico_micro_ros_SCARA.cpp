#include <iostream>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
extern "C" {
    #include "pico_uart_transports.h" // Archivo de encabezado correspondiente
}
#include <std_msgs/msg/float32.h>


volatile long encoder_count = 0;
volatile long encoder_count2 = 0;
//Definicion global pin Led deafult
#define LED_PIN PICO_DEFAULT_LED_PIN

//Defino pi para posterior uso
#define PI 3.14159265358979323846

float velocidad_1=0;
float velocidad_2=0;
float Pos_Rad_1=0;
float Pos_Rad_2=0;

//---------------------------------------------------------------------             Clase Motor                ----------------------------------------------------------------------------------------
class Motor {
    private : //atributos
    int pin_num_pwm, pin_num_in1, pin_num_in2,sensorA_pin, sensorB_pin, freq=1000 , FC1 , FC2;
    //parametros
    int ppr;
    int decoder_number;
    int gear_ratio;

    absolute_time_t prevT = get_absolute_time();
    double posPrev;
    int slice_num;
    int CH;
    
    public://metodos
    

        //Metodo constructor
        Motor(int _pin_num_pwm, int _pin_num_in1, int _pin_num_in2, int _sensorA_pin, int _sensorB_pin, int _freq, int _FC1 , int _FC2){

        this-> pin_num_pwm =    _pin_num_pwm;
        this-> pin_num_in1 =    _pin_num_in1;
        this-> pin_num_in2 =    _pin_num_in2;
        this-> sensorA_pin =    _sensorA_pin;
        this-> sensorB_pin =    _sensorB_pin;
        this-> freq        =    _freq;
        this -> FC1        =    _FC1;
        this -> FC2        =    _FC2;
                
        gpio_init(this->pin_num_pwm);
        gpio_init(this->pin_num_in1);
        gpio_init(this->pin_num_in2);
        gpio_init(this->sensorA_pin);
        gpio_init(this->sensorB_pin);
        gpio_init(this->FC1);
        gpio_init(this->FC2);
        
        gpio_set_function(this->pin_num_pwm, GPIO_FUNC_PWM);
        gpio_set_dir(this->pin_num_in1, GPIO_OUT);
        gpio_set_dir(this->pin_num_in2, GPIO_OUT);
        gpio_set_dir(this->sensorA_pin, GPIO_IN);
        gpio_set_dir(this->sensorB_pin, GPIO_IN);
        gpio_set_dir(this->FC1, GPIO_IN);
        gpio_set_dir(this->FC2, GPIO_IN);
        gpio_pull_down(this->FC1);  
        gpio_pull_down(this->FC2);  
      
        this->slice_num = pwm_gpio_to_slice_num(this->pin_num_pwm);
        this->CH = pwm_gpio_to_channel(this->pin_num_pwm);
        pwm_set_clkdiv(slice_num,10.0);
        pwm_set_wrap(slice_num, 65535);
        pwm_set_enabled(slice_num,true);
        
        
        // Encoder and motor parameters
        this-> ppr = 16;
        this-> decoder_number = 2;
        this-> gear_ratio = 50;


        // Speed time variables
        this-> posPrev = 0;
        
        
      
        
    }
    
    
    
    
    // Metodo get para obtener pin Sensor B
    int getsensorB_pin() {
        return this->sensorB_pin;
    }
    int getsensorA_pin() {
        return this->sensorA_pin;
    }

    //Funcion para asignar direccion y ciclo de dureza
    void set_motor(int dir,int speed){
        if (dir==1){
            gpio_put(this->pin_num_in1,0);
            gpio_put(this->pin_num_in2,1);
            
            
        }
        else if (dir==-1){
            gpio_put(this->pin_num_in1,1);
            gpio_put(this->pin_num_in2,0);
        }
        else{
            gpio_put(this->pin_num_in1,0);
            gpio_put(this->pin_num_in2,0);
        }
        
        pwm_set_chan_level(this->slice_num,this->CH, speedToU16(speed));
            
    }
    
    //Metodo conversor de porcentaje 0-100 a 16 bits
    
    int speedToU16(int speed){
        int speed_u16=speed*65535.0/100;
        return speed_u16;
        
    }
    
    //Metodo para obtener la velocidad del motor

    double encoder2speed(int encoder_count_){
        absolute_time_t currT =get_absolute_time();
        
        int64_t deltaT = absolute_time_diff_us(this-> prevT, currT);
        this -> prevT=currT;
        double resta=encoder_count_-this->posPrev;
        double vel = ((resta*1000000)/deltaT);
        this-> posPrev =encoder_count_;
        double vel_rpm_raw = ((vel)/(this->ppr*this->decoder_number*this->gear_ratio))*60;
        return vel_rpm_raw;
        
    }
    //Posicion del motor
    float encoder2pos_deg(int encoder_count_){
        
        float pos = encoder_count_*360/(this->ppr * this->gear_ratio * this-> decoder_number);
        
        return pos;
    }
    float encoder2pos_rad(int encoder_count_){
        
        float pos = encoder_count_*2*PI/(this->ppr * this->gear_ratio* this->decoder_number);
        
        return pos;
    }
    
    //Valor del Finales de carrera
    
    int FC_value (){
        
        int FC=0;
        int V_FC1=gpio_get(this->FC1);
        int V_FC2=gpio_get(this->FC2);
        
        if (V_FC1==1){
            FC=1;
        }
        
        else if (V_FC2==1){
            FC=2;   
        }
        
        else {
            FC=0;
        }
        
    return FC;
        
    }

};

    Motor motor_1(20, 18, 16, 9,10, 1000,27,20);
    Motor motor_2(17, 21, 19, 13,15, 1000,7,8);

    int sensorB_M1=motor_1.getsensorB_pin(); 
    int  sensorA_M1= motor_1.getsensorA_pin();
    int sensorB_M2=motor_2.getsensorB_pin(); 
    int  sensorA_M2= motor_2.getsensorA_pin();


//----------------------------------------------------------------------          Definicion funciones Callback Interrupciones           --------------------------------------------------    
static void encoderAInterruptHandler(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(gpio_get(sensorB_M1>0)){
    encoder_count--;
    }
    else{
    encoder_count++;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void encoderBInterruptHandler(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(gpio_get(sensorA_M1>0)){
    encoder_count++;
    }
    else{
    encoder_count--;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void encoderAInterruptHandler2(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(gpio_get(sensorB_M2>0)){
    encoder_count2--;
    }
    else{
    encoder_count2++;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void encoderBInterruptHandler2(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(gpio_get(sensorA_M2>0)){
    encoder_count2++;
    }
    else{
    encoder_count2--;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}




int ms_to_ticks(int ms){

    int ticks= (ms*100)/1000;

    return ticks;
}

//----------------------------------------------------------------------------------- Definicion de Publicadores , Mensajes etc ----------------------------------------------------------------------

rcl_publisher_t publisher_Pos_1;
std_msgs__msg__Float32 msg_Pos_1;

rcl_publisher_t publisher_Pos_2;
std_msgs__msg__Float32 msg_Pos_2;

rcl_node_t node;
rcl_allocator_t allocator; //Distribuidor de memoria
rclc_support_t support; 
rclc_executor_t executor;  //Distribuidor de Tareas_uROS




//-----------------------------------------------------------------------------------  Tareas de FREERTOS ---------------------------------------------------------------------------------------------------

void stateMachine(void *pvParameters)
{   
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
//Variables
        int P1;
        int F1_V;
        int Dir_HomeR; //direccion -1
        int Dir_HomeR2; //direccion -1
        int led_value=0;

    //Estados
    int Estado=0; //unconfigured
    int speed=19;


    while (true) {

        switch (Estado) {
            case 0: //unconfigured

                // Defino Boton de Inicio
                //P1=27;
                //gpio_init(P1);
                //gpio_set_dir(P1, GPIO_IN);
                //gpio_pull_down(P1); 
                //F1_V=0;

                //Variables
                Dir_HomeR=0; 
                led_value=0;
                Dir_HomeR2=0; 
                //Inicializacion de pin 26 como entrada ADC
                //adc_gpio_init(26);
                //adc_select_input(0);
                //Asignacion de Led default como salida
                //gpio_init(LED_PIN);
                //gpio_set_dir(LED_PIN,GPIO_OUT);
                //F1_V =gpio_get(P1);

             
                

                vTaskDelay(ms_to_ticks(5000));
                Estado=3;
            
            break;



            case 1: //Home rutine
                

                if (Dir_HomeR==0){ //Direccion 0
               
                    int FC_P=motor_1.FC_value();

                    if (FC_P==1){
                        Dir_HomeR=1;
                        
                    }
                    else{
                    motor_1.set_motor(1, speed);
                    }
                }
        
                if (Dir_HomeR==1){ //Direccion 1
                    int FC_P=motor_1.FC_value();
                    
                    if (FC_P==2){
                       
                        motor_1.set_motor(1, speed);
                        vTaskDelay(ms_to_ticks(1000));
                        Estado=2;
                    }

                    else{
                    motor_1.set_motor(-1, speed);
                    }
                }
                vTaskDelay(ms_to_ticks(50));
            break;

            case 2:
                
                motor_1.set_motor(1, 1);

                if (Dir_HomeR2==0){ //Direccion 0
               
                    int FC_P2=motor_2.FC_value();

                    if (FC_P2==1){
                        
                        Dir_HomeR2=1;
                    }
                    else{
                    motor_2.set_motor(1, speed);
                    }
                }
        
                if (Dir_HomeR2==1){ //Direccion 1
                    int FC_P2=motor_2.FC_value();
                 
                    if (FC_P2==2){
                     
                        motor_2.set_motor(1, speed);
                        vTaskDelay(ms_to_ticks(500));
                        motor_2.set_motor(1, 1);
                        Estado=3;
                    }

                    else{
                    motor_2.set_motor(-1, speed);
                    }
                }
                vTaskDelay(ms_to_ticks(50));

            break;




            case 3: //active

                led_value=1-led_value;
                gpio_put(LED_PIN,led_value);

                motor_1.set_motor(-1, 100);
                vTaskDelay(ms_to_ticks(700));

            break;
        }
}
}


void Print_Task(void *pvParameters) {
    while (1) {
                velocidad_1 =motor_1.encoder2speed(encoder_count);
                float Pos_deg=motor_1.encoder2pos_deg(encoder_count);
                Pos_Rad_1=motor_1.encoder2pos_rad(encoder_count);
                velocidad_2 =motor_2.encoder2speed(encoder_count2);
                float Pos_deg_2=motor_2.encoder2pos_deg(encoder_count2);
                Pos_Rad_2=motor_2.encoder2pos_rad(encoder_count2);
                msg_Pos_2.data=Pos_Rad_2;
                msg_Pos_1.data=Pos_Rad_1;

                rcl_publish(&publisher_Pos_2, &msg_Pos_2, NULL);
                rcl_publish(&publisher_Pos_1, &msg_Pos_1, NULL);

        

        vTaskDelay(ms_to_ticks(1000));
    }
}


void Spin_Task(void *pvParameters) {

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        vTaskDelay(ms_to_ticks(50));
    }

}







//-------------------------------------------------------------------------- Main ---------------------------------------------------------------------------------------------




int main()
{


    gpio_set_irq_enabled_with_callback(sensorB_M1, GPIO_IRQ_EDGE_RISE, 1, encoderAInterruptHandler);
    gpio_set_irq_enabled_with_callback(sensorA_M1, GPIO_IRQ_EDGE_RISE, 1, encoderBInterruptHandler);
    gpio_set_irq_enabled_with_callback(sensorB_M2, GPIO_IRQ_EDGE_RISE, 1, encoderAInterruptHandler2);
    gpio_set_irq_enabled_with_callback(sensorA_M2, GPIO_IRQ_EDGE_RISE, 1, encoderBInterruptHandler2);

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	); // DEFINIR TIPO DE TRANSPORTE USB

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);


    allocator = rcl_get_default_allocator(); //inicializar el allocator

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);



        if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }  



    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_publisher_init_default(
        &publisher_Pos_2,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,Float32),
        "publisher_Pos_2");

  rclc_publisher_init_default(
        &publisher_Pos_1,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,Float32),
        "publisher_Pos_1");


    rclc_executor_init(&executor, &support.context, 2, &allocator);
    gpio_put(LED_PIN, 1);

    xTaskCreate(Print_Task, "Imprimir", 256, NULL, 1, NULL);
    xTaskCreate(stateMachine, "Maquina de estados", 256, NULL, 2, NULL);
    xTaskCreate(Spin_Task, "Spin", 256, NULL, 4, NULL);
    vTaskStartScheduler();

    return 0;
}
