# inclua  < webots / robot.h >
# inclui  < webots / motor.h >
# inclui  < webots / distance_sensor.h >
# include  < webots / position_sensor.h >
# inclua  < webots / supervisor.h >

# inclui  < stdio.h >
# inclui  < stdlib.h >
# inclui  < string.h >

# define  TIME_STEP  64
# define  MAX_SPEED  6.28

# define  WHEEL_DISTANCE  0,052     // Distância entre rodas em metros (comprimento do eixo); é o mesmo valor de "WHEEL_SEPARATION", mas expresso em metros.
# define  WHEEL_RADIUS  0,0205     // m.
# define  WHEEL_CIRCUMFERENCE WHEEL_RADIUS * M_PI * 2.0     // Circunferência da roda (metros).
# define  ROBOT_RADIUS  0,035  // metros.
# define  RANGE_MIN  0,005 + ROBOT_RADIUS // 0,5 cm + ROBOT_RADIUS.
# define  RANGE_MAX  0,05 + ROBOT_RADIUS // 5 cm + ROBOT_RADIUS.

// posições de proximidade do e-puck (cm), x apontando para frente, y apontando para a esquerda
//            P7 (3,5, 1,0) P0 (3,5, -1,0)
//        P6 (2,5, 2,5) P1 (2,5, -2,5)
//    P5 (0,0, 3,0) P2 (0,0, -3,0)
//        P4 (-3,5, 2,0) P3 (-3,5, -2,0)

// orentações de proximidade do e-puck (graus)
//            P7 (10) P0 (350)
//        P6 (40) P1 (320)
//    P5 (90) P2 (270)
//        P4 (160) P3 (200)

typedef  struct {

WbDeviceTag right_motor;  
WbDeviceTag left_motor;
WbFieldRef robot_position;
WbFieldRef robot_rotation;
WbDeviceTag ps [ 8 ];

} Robot_concepts;

Robot_concepts robot_init () {
  
  Conceitos de conceitos de robô;
  wb_robot_init ();
  conceitos. left_motor = wb_robot_get_device ( " motor da roda esquerda " );
  conceitos. motor_direito = wb_robot_get_device ( " motor da roda direita " );
  wb_motor_set_position (conceitos. left_motor , INFINITY);
  wb_motor_set_position (conceitos. right_motor , INFINITY);
  wb_motor_set_velocity (conceitos. left_motor , 0,1 * MAX_SPEED);
  wb_motor_set_velocity (conceitos. right_motor , 0,1 * MAX_SPEED);
  WbDeviceTag left_encoder = wb_robot_get_device ( " sensor da roda esquerda " );
  WbDeviceTag right_encoder = wb_robot_get_device ( " sensor da roda direita " );
  wb_position_sensor_enable (left_encoder, TIME_STEP);
  wb_position_sensor_enable (right_encoder, TIME_STEP);
    char ps_id [ 4 ];
    para ( int i = 0 ; i < 8 ; i ++) {
      sprintf (ps_id, " ps % d " , i);
      conceitos. ps [i] = wb_robot_get_device (ps_id);
      wb_distance_sensor_enable (conceitos. ps [i], TIME_STEP);
    }
  WbNodeRef robot_node = wb_supervisor_node_get_from_def ( " EPUCK " );
    if (robot_node == NULL ) {
    fprintf (stderr, " Nenhum nó DEF EPUCK encontrado no arquivo mundial atual \ n " );
    saída ( 1 );
    }
  conceitos. robot_position = wb_supervisor_node_get_field (robot_node, " tradução " );
  conceitos. robot_rotation = wb_supervisor_node_get_field (robot_node, " rotação " );

  conceitos de retorno ;
}
  
bool  detect_obstacle_ahead ( float d [ 8 ]) {
  return ((d [ 0 ] <RANGE_MAX) ||
           (d [ 1 ] <RANGE_MAX) ||
           (d [ 6 ] <RANGE_MAX) ||
           (d [ 7 ] <RANGE_MAX));
}

float  convert_intensity_to_meters ( float prox) {
  float dist = 0,0 ;
  if (prox> 0,0 )
    dist = 0,5 / sqrt (prox) + ROBOT_RADIUS;
  outro
    dist = RANGE_MAX;
  if (dist> RANGE_MAX)
    dist = RANGE_MAX;
  return dist;
}
 
void  salvar_posicao_distancias (FILE * log, float x, float y, float angulo, float distancia [ 8 ]) {
    fprintf ( log , " % f  % f  % f  " , x, y, angulo);
    para ( int i = 0 ; i < 8 ; i ++) {
      fprintf ( log , " % f  " , distancia [i]);
    }
    fprintf ( log , " \ n " );
    fflush ( log );
}

int  main ( int argc, char ** argv) {
  ARQUIVO * log = fopen ( " log.csv " , " w " );
  if (! log )
    saída ( 1 );
    
   // // EXERCICIO: CRIAR FUNÇÃO PARA SUBSTITUIR ESSE CODIGO USANDO STRUCT /////
  
   // wb_robot_init ();
   // WbDeviceTag left_motor = wb_robot_get_device ("motor da roda esquerda");
   // WbDeviceTag right_motor = wb_robot_get_device ("motor da roda direita");
   // wb_motor_set_position (left_motor, INFINITY);
   // wb_motor_set_position (right_motor, INFINITY);
   // wb_motor_set_velocity (left_motor, 0,1 * MAX_SPEED);
   // wb_motor_set_velocity (right_motor, 0,1 * MAX_SPEED);
   // WbDeviceTag left_encoder = wb_robot_get_device ("sensor da roda esquerda");
   // WbDeviceTag right_encoder = wb_robot_get_device ("sensor da roda direita");
   // wb_position_sensor_enable (left_encoder, TIME_STEP);
   // wb_position_sensor_enable (right_encoder, TIME_STEP);
   // WbDeviceTag ps [8];
   // char ps_id [4];
   // para (int i = 0; i <8; i ++) {
   // sprintf (ps_id, "ps% d", i);
   // ps [i] = wb_robot_get_device (ps_id);
   // wb_distance_sensor_enable (ps [i], TIME_STEP);
   // }
   // WbNodeRef robot_node = wb_supervisor_node_get_from_def ("EPUCK");
   // if (robot_node == NULL) {
   // fprintf (stderr, "Nenhum nó DEF EPUCK encontrado no arquivo mundial atual \ n");
   // exit (1);
   // }
   // WbFieldRef robot_position = wb_supervisor_node_get_field (robot_node, "tradução");
   // WbFieldRef robot_rotation = wb_supervisor_node_get_field (robot_node, "rotação");
  
   // // EXERCICIO: CRIAR FUNÇÃO PARA SUBSTITUIR ESSE CODIGO USANDO STRUCT /////
  Robot_concepts settings = robot_init ();
  float dist [ 8 ];
  float x = 0 , y = 0 , teta = 0 ;
  float left_steps_prev = 0 , right_steps_prev = 0 ;
  enquanto ( wb_robot_step (TIME_STEP)! = - 1 ) {
    const  double * position = wb_supervisor_field_get_sf_vec3f (configurações. robot_position );
    const  duplo * rotação = wb_supervisor_field_get_sf_rotation (configurações. robot_rotation );

    para ( int i = 0 ; i < 8 ; i ++) {
      dist [i] = convert_intensity_to_meters ( wb_distance_sensor_get_value (configurações. ps [i]));
    }

    salvar_posicao_distancias ( log , posição [ 0 ], posição [ 2 ], rotação [ 3 ], dist);

    if ( detect_obstacle_ahead (dist))
    {
      wb_motor_set_velocity (configurações. left_motor , 0,2 * MAX_SPEED);
      wb_motor_set_velocity (configurações. right_motor , - 0,2 * MAX_SPEED);
    }
    if (! detect_obstacle_ahead (dist))
    {
      wb_motor_set_velocity (configurações. left_motor , 0,5 * MAX_SPEED);
      wb_motor_set_velocity (configurações. right_motor , 0,5 * MAX_SPEED);
    }
  }
   
  fclose ( log );

  wb_robot_cleanup ();

  return  0 ;
}
