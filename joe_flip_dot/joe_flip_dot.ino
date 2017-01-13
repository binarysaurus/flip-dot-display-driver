int clockf = 4; // relay 4
int selectf = 5; // relay 3
int resetf = 6; // relay 2

bool array[8] = {0,1,0,0,1,0,0,1};

void setup() {
  Serial.begin(9600);  //set 9600 baud
  pinMode(clockf, OUTPUT); //clock
  pinMode(selectf, OUTPUT); //select
  pinMode(resetf, OUTPUT); //reset
  digitalWrite(resetf, HIGH);
  delay(54);
  digitalWrite(resetf, LOW);

  for (int i = 0; i < 8; i++)
   {
       
     digitalWrite(selectf, !array[i]);
     delay(5);
     digitalWrite(clockf, HIGH);
     delay(50);
     digitalWrite(clockf, LOW);
     delay(50);

   }
   digitalWrite(selectf, LOW);
}

void loop() {

//NOOO
}
