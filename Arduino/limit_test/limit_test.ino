

#define ARM_R 0
#define ARM_L 1

#define ARM_R_J1_DIR 5
#define ARM_R_J2_DIR 6
#define ARM_L_J1_DIR 7
#define ARM_L_J2_DIR 13

#define ARM_R_J1_STEP 2
#define ARM_R_J2_STEP 3
#define ARM_L_J1_STEP 4
#define ARM_L_J2_STEP 12

#define EN 8 //stepper motor enable , active low
#define STEPS_PER_180 3500
#define step2angle(x) (x*180.0/STEPS_PER_180)
#define abs(x) (x>=0?x:-x)
#define J1_L_LIMIT 200
#define J1_U_LIMIT 3000
#define J2_L_LIMIT 200
#define J2_U_LIMIT 7000

int arm_r_j1_cstep = 0;
int arm_r_j2_cstep = 0;
int arm_l_j1_cstep = 0;
int arm_l_j2_cstep = 0;
float angles[4] = {0, 0, 0, 0};

// Geometry definitions
#define setup_D = 50.0  //Distance between J1 of arms [cm]
#define setup_L1 = 25.3 //Lengthh of beam between J1 and J2 [cm]
#define setup_L2 = 21.0 //Length of beam between J2 and arm end [cm]

void move_r_to(int arm, float x, float y) {
    float L1 = 25.3; // Length from J1 to J2
    float L2 = 21.0; // Length from J2 to end-effector
    float theta1, theta2;
    

    // Compute inverse kinematics using geometric approach
    float d = sqrt(x*x + y*y);
    Serial.println(d);
    if (d > (L1 + L2) || d < fabs(L1 - L2)) {
        Serial.println("Position out of reach");
        return;
    }
    
        float cos_theta2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
    theta2 = atan2(sqrt(1 - cos_theta2*cos_theta2), cos_theta2); // Elbow down
    float k1 = L1 + L2 * cos(theta2);
    float k2 = L2 * sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);
    
    // Convert reference angles from degrees to radians
    float reference_theta1 = 68.86 * M_PI / 180.0;
    float reference_theta2 = 181.03 * M_PI / 180.0;
    
    theta1 = theta1 - M_PI/2 + reference_theta1;
    theta2 += reference_theta2;
    
    // Convert angles to degrees before converting to steps
    theta1 = theta1 * 180.0 / M_PI;
    theta2 = theta2 * 180.0 / M_PI;
    //Serial.print(theta1); Serial.print(","); Serial.println(theta2);
    float delta_aj1 = theta1 - step2angle(arm_r_j1_cstep);
    float delta_aj2 = theta2 - step2angle(arm_r_j2_cstep);
    step((delta_aj1<0), ARM_R_J1_DIR, ARM_R_J1_STEP, (int)(abs(delta_aj1)/180*STEPS_PER_180), true);
    step((delta_aj2<0), ARM_R_J2_DIR, ARM_R_J2_STEP, (int)(abs(delta_aj2)/180*STEPS_PER_180), true);
}

void move_l_to(int arm, float x, float y) {
    float L1 = 25.3; // Length from J1 to J2
    float L2 = 21.0; // Length from J2 to end-effector
    float theta1, theta2;
    x = -x;

    // Compute inverse kinematics using geometric approach
    float d = sqrt(x*x + y*y);
    Serial.println(d);
    if (d > (L1 + L2) || d < fabs(L1 - L2)) {
        Serial.println("Position out of reach");
        return;
    }
    
    float cos_theta2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
    theta2 = atan2(sqrt(1 - cos_theta2*cos_theta2), cos_theta2); // Elbow down
    float k1 = L1 + L2 * cos(theta2);
    float k2 = L2 * sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);
    
    // Convert reference angles from degrees to radians
    float reference_theta1 = 68.86 * M_PI / 180.0;
    float reference_theta2 = 181.03 * M_PI / 180.0;
    
    theta1 = theta1 - M_PI/2 + reference_theta1;
    theta2 += reference_theta2;
    
    // Convert angles to degrees before converting to steps
    theta1 = theta1 * 180.0 / M_PI;
    theta2 = theta2 * 180.0 / M_PI;
    //Serial.print(theta1); Serial.print(","); Serial.println(theta2);
    float delta_aj1 = theta1 - step2angle(arm_l_j1_cstep);
    float delta_aj2 = theta2 - step2angle(arm_l_j2_cstep);
    step((delta_aj1>0), ARM_L_J1_DIR, ARM_L_J1_STEP, (int)(abs(delta_aj1)/180*STEPS_PER_180), true);
    step((delta_aj2>0), ARM_L_J2_DIR, ARM_L_J2_STEP, (int)(abs(delta_aj2)/180*STEPS_PER_180), true);
}

/*
// Function : step . function: to control the direction of the stepper motor , the number of steps .
// Parameters : dir direction control , dirPin corresponding stepper motor DIR pin , stepperPin corresponding 
// stepper motor " step " pin , Step number of step of no return value.
*/

void step (boolean dir, byte dirPin, byte stepperPin, int steps, boolean use_limits)
{
  int i;
  /*
  Serial.print("START: (RJ1:"); Serial.print(step2angle(arm_r_j1_cstep));
  Serial.print("),(RJ2:"); Serial.print(step2angle(arm_r_j2_cstep));
  Serial.print("(LJ1:"); Serial.print(step2angle(arm_l_j1_cstep));
  Serial.print("),(LJ2:"); Serial.print(step2angle(arm_l_j2_cstep));
  Serial.println(")");
  */

  digitalWrite (EN, LOW);
  digitalWrite (dirPin, dir);
  delay (50);
  for (int i = 0; i<steps;i++) 
  {
    digitalWrite (stepperPin, HIGH);
    delayMicroseconds (800);
    digitalWrite (stepperPin, LOW);
    switch(dirPin) {
      case ARM_R_J1_DIR: {
        arm_r_j1_cstep += (dir?-1:1);
        break;
      }
      case ARM_R_J2_DIR: {
        arm_r_j2_cstep += (dir?-1:1); 
        break;
      }
      case ARM_L_J1_DIR: {
        arm_l_j1_cstep += (dir?1:-1); 
        break;
      }
      case ARM_L_J2_DIR: {
        arm_l_j2_cstep += (dir?1:-1); 
        break;
      }                  
      default: break;
    }
    if(use_limits)
      switch(dirPin) {
          case ARM_R_J1_DIR: {
            if( (arm_r_j1_cstep<J1_L_LIMIT) || (arm_r_j1_cstep > J1_U_LIMIT) ) {digitalWrite (EN, HIGH); return;}
            break;
          }
          case ARM_R_J2_DIR: {
            if( (arm_r_j2_cstep<J2_L_LIMIT) || (arm_r_j2_cstep > J2_U_LIMIT) ) {digitalWrite (EN, HIGH); return;}
            break;
          }
          case ARM_L_J1_DIR: {
            if( (arm_l_j1_cstep<J1_L_LIMIT) || (arm_l_j1_cstep > J1_U_LIMIT) ) {digitalWrite (EN, HIGH); return;}
            break;
          }
          case ARM_L_J2_DIR: {
            if( (arm_l_j2_cstep<J2_L_LIMIT) || (arm_l_j2_cstep > J2_U_LIMIT) ) {digitalWrite (EN, HIGH); return;}
            break;
          }                  
          default: break;
      }
    delayMicroseconds (800);
    }
  digitalWrite (EN, HIGH);
  angles[0] = step2angle(arm_r_j1_cstep);
  angles[1] = step2angle(arm_r_j2_cstep);
  angles[2] = step2angle(arm_l_j1_cstep);
  angles[3] = step2angle(arm_l_j2_cstep);  

  /*
  Serial.print("STOP: (RJ1:"); Serial.print(step2angle(arm_r_j1_cstep));
  Serial.print("),(RJ2:"); Serial.print(step2angle(arm_r_j2_cstep));
  Serial.print("(LJ1:"); Serial.print(step2angle(arm_l_j1_cstep));
  Serial.print("),(LJ2:"); Serial.print(step2angle(arm_l_j2_cstep));
  Serial.println(")");
  */
}

void find_limit (boolean dir, byte dirPin, byte stepperPin)
{
  int x = digitalRead(9);
  int y = digitalRead(10);
  int z = digitalRead(11);
  digitalWrite (dirPin, dir);
  delay (50);
  x = digitalRead(9);
  y = digitalRead(10);
  z = digitalRead(11);
  while( (x+y+z) == 3)
  {
    digitalWrite (stepperPin, HIGH);
    delayMicroseconds (800);
    digitalWrite (stepperPin, LOW);
    delayMicroseconds (800);
    x = digitalRead(9);
    y = digitalRead(10);
    z = digitalRead(11);
    delay(1);
  }
  int i;
  digitalWrite (dirPin, (dir==true?false:true));
  delay (50);
  for(i=0;i<100;i++)
  {
    digitalWrite (stepperPin, HIGH);
    delayMicroseconds (800);
    digitalWrite (stepperPin, LOW);
    delayMicroseconds (800);
    x = digitalRead(9);
    y = digitalRead(10);
    z = digitalRead(11);
    delay(1);
  }
  
}

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600); // Start serial communication at 9600 baud
    while (!Serial) {
        ; // Wait for the serial connection
    }
    Serial.println("Send commands starting with #IAA for all angles, or individual commands: #RJ1, #RJ2, #LJ1, #LJ2. Query with ?RJ1, ?RJ2, ?LJ1, ?LJ2.");
  pinMode(9,INPUT_PULLUP);
  pinMode(10,INPUT_PULLUP);
  pinMode(11,INPUT_PULLUP);

  pinMode (ARM_R_J1_DIR, OUTPUT); pinMode (ARM_R_J1_STEP, OUTPUT);
  pinMode (ARM_L_J1_DIR, OUTPUT); pinMode (ARM_L_J1_STEP, OUTPUT);
  pinMode (ARM_R_J2_DIR, OUTPUT); pinMode (ARM_R_J2_STEP, OUTPUT);
  pinMode (ARM_L_J2_DIR, OUTPUT); pinMode (ARM_L_J2_STEP, OUTPUT);


  pinMode (EN, OUTPUT);
  digitalWrite (EN, HIGH);

  digitalWrite (EN, LOW);
  delay(50);
  find_limit(true, ARM_R_J2_DIR, ARM_R_J2_STEP);
  find_limit(false, ARM_L_J2_DIR, ARM_L_J2_STEP);
  find_limit(true, ARM_R_J1_DIR, ARM_R_J1_STEP);
  find_limit(false, ARM_L_J1_DIR, ARM_L_J1_STEP);

  arm_r_j1_cstep = 100;
  arm_r_j2_cstep = 100;
  arm_l_j1_cstep = 100;
  arm_l_j2_cstep = 100;

  step(true, ARM_L_J1_DIR, ARM_L_J1_STEP, 1200, false);
  step(false, ARM_R_J1_DIR, ARM_R_J1_STEP, 1200, false);
  step(true, ARM_L_J2_DIR, ARM_L_J2_STEP, 3420, false);
  step(false, ARM_R_J2_DIR, ARM_R_J2_STEP, 3420, false);
  Serial.print("(RJ1:"); Serial.print(step2angle(arm_r_j1_cstep));
  Serial.print("),(RJ2:"); Serial.print(step2angle(arm_r_j2_cstep));
  Serial.print("(LJ1:"); Serial.print(step2angle(arm_l_j1_cstep));
  Serial.print("),(LJ2:"); Serial.print(step2angle(arm_l_j2_cstep));
  Serial.println(")");
  Serial.println();
  digitalWrite (EN, HIGH);

  angles[0] = step2angle(arm_r_j1_cstep);
  angles[1] = step2angle(arm_r_j2_cstep);
  angles[2] = step2angle(arm_l_j1_cstep);
  angles[3] = step2angle(arm_l_j2_cstep);  
  Serial.println("Ok");  
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n'); // Read the input until newline
        Serial.println(input);
        input.trim();
        if (input.startsWith("#IAA")) {
            input.remove(0, 4); // Remove command identifier
            int index = 0;
            char *ptr;
            char buffer[input.length() + 1];
            input.toCharArray(buffer, sizeof(buffer));
            
            ptr = strtok(buffer, ",");
            while (ptr != NULL && index < 4) {
                angles[index] = atof(ptr); // Convert string to float
                index++;
                ptr = strtok(NULL, ",");
            }
            
            if (index == 4) {
                Serial.println("Angles updated successfully.");
            } else {
                Serial.println("Error: Please enter exactly four angles separated by commas.");
            }
        } else if (input.startsWith("#RJ1")) {
            angles[0] = input.substring(4).toFloat();
            float delta_a = angles[0] - step2angle(arm_r_j1_cstep);
            step((delta_a<0), ARM_R_J1_DIR, ARM_R_J1_STEP, (int)(abs(delta_a)/180*STEPS_PER_180), true);
            Serial.println("Ok");
        } else if (input.startsWith("#RAC")) {
            input.remove(0, 4); // Remove command identifier
            int index = 0;
            float x = 0;
            float y = 0; 
            char *ptr;
            char buffer[input.length() + 1];
            input.toCharArray(buffer, sizeof(buffer));
            
            ptr = strtok(buffer, ",");
            while (ptr != NULL && index < 2) {
                if(index == 0) x = atof(ptr); else y = atof(ptr); // Convert string to float
                index++;
                ptr = strtok(NULL, ",");
            }
            
            if (index == 2) {
                move_r_to(ARM_R, x,y);
                //Serial.print(x); Serial.print(","); Serial.println(y);
                Serial.println("Ok");
            } else {
                Serial.println("Error: Please enter exactly four angles separated by commas.");
            }
        }  else if (input.startsWith("#LAC")) {
            input.remove(0, 4); // Remove command identifier
            int index = 0;
            float x = 0;
            float y = 0; 
            char *ptr;
            char buffer[input.length() + 1];
            input.toCharArray(buffer, sizeof(buffer));
            
            ptr = strtok(buffer, ",");
            while (ptr != NULL && index < 2) {
                if(index == 0) x = atof(ptr); else y = atof(ptr); // Convert string to float
                index++;
                ptr = strtok(NULL, ",");
            }
            
            if (index == 2) {
                move_l_to(ARM_L, x,y);
                //Serial.print(x); Serial.print(","); Serial.println(y);
                Serial.println("Ok");
            } else {
                Serial.println("Error: Please enter exactly four angles separated by commas.");
            }
        }  else if (input.startsWith("#RJ2")) {
            angles[1] = input.substring(4).toFloat();
            float delta_a = angles[1] - step2angle(arm_r_j2_cstep);
            step((delta_a<0), ARM_R_J2_DIR, ARM_R_J2_STEP, (int)(abs(delta_a)/180*STEPS_PER_180), true);    
            Serial.println("Ok");        
        } else if (input.startsWith("#LJ1")) {
            angles[2] = input.substring(4).toFloat();
            float delta_a = angles[2] - step2angle(arm_l_j1_cstep);
            step((delta_a>0), ARM_L_J1_DIR, ARM_L_J1_STEP, (int)(abs(delta_a)/180*STEPS_PER_180), true);     
            Serial.println("Ok");       
        } else if (input.startsWith("#LJ2")) {
            angles[3] = input.substring(4).toFloat();
            float delta_a = angles[3] - step2angle(arm_l_j2_cstep);
            step((delta_a>0), ARM_L_J2_DIR, ARM_L_J2_STEP, (int)(abs(delta_a)/180*STEPS_PER_180), true);  
            Serial.println("Ok");          
        } else if (input == "?RJ1") {
            Serial.println(angles[0]);
        } else if (input == "?RJ2") {
            Serial.println(angles[1]);
        } else if (input == "?LJ1") {
            Serial.println(angles[2]);
        } else if (input == "?LJ2") {
            Serial.println(angles[3]);
        } else {
            Serial.println("Error: Unknown command.");
        }
    }
}

/*
void loop() {
  // put your main code here, to run repeatedly:
  int x = digitalRead(9);
  int y = digitalRead(10);
  int z = digitalRead(11);
  Serial.print(x); Serial.print(",");
  Serial.print(y); Serial.print(",");
  Serial.println(z); 

  digitalWrite (EN, LOW);
  //step(false, ARM_L_J1_DIR, ARM_L_J1_STEP, 100);
  //step(false, ARM_R_J1_DIR, ARM_R_J1_STEP, 100);
  //step(false, ARM_L_J2_DIR, ARM_L_J2_STEP, 30);
  //step(false, ARM_R_J2_DIR, ARM_R_J2_STEP, 300);
  digitalWrite (EN, HIGH);

  delay(1000);
  digitalWrite (EN, LOW);
  //step(true, ARM_L_J1_DIR, ARM_L_J1_STEP, 100); //true is "myötäpäivään"
  //step(true, ARM_R_J1_DIR, ARM_R_J1_STEP, 100); //true is "myötäpäivään"
  //step(true, ARM_L_J2_DIR, ARM_L_J2_STEP, 300); //true is "myötäpäivään"
  //step(true, ARM_R_J2_DIR, ARM_R_J2_STEP, 300); //true is "myötäpäivään"
  digitalWrite (EN, HIGH);

  delay(2000);
}
*/

