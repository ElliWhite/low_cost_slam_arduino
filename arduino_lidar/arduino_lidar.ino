int LED_BUILTIN = 2;
byte Sync0[1];
byte Sync1[1];
byte Sync[2];
byte lidarData[2520];
byte remainingData[2518];
int received_length = 0;
byte chksm = 0;
int good_packet = 0;
int packet_count = 0;

HardwareSerial & lidar = Serial2;

void setup() {

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  lidar.begin(230400);
  lidar.write('b');

}


void loop() {

  digitalWrite(LED_BUILTIN, HIGH);

  // keep reading data until find very first packets 0xFA, 0xA0
  while((Sync0[0] != 0xFA) || (Sync1[0] != 0xA0)) {
    
    // keep reading single byte until finding sync byte "0xFA"
    received_length = lidar.readBytes(Sync0, 1);
    
    // read second byte if first sync byte found
    if(Sync0[0] == 0xFA){
      received_length = lidar.readBytes(Sync1, 1);
    }

  }

  // now we have first two sync byte of lidar, read remaining data.
  // 2520 total bytes (42 bytes and 60 packets) (2518 + 2 sync)
  if(Sync0[0] == 0xFA && Sync1[0] == 0xA0){
    received_length = lidar.readBytes(remainingData, 2518);
  }

  Sync[0] = Sync0[0];
  Sync[1] = Sync1[0];

  memcpy(lidarData, Sync, sizeof(Sync));
  memcpy(lidarData+sizeof(Sync), remainingData, sizeof(remainingData));

  // check sum. Add first 40 bytes of packet, then do 0xFF - chksm. If equals either byte 41 or 42 in packet, data valid.
  for(int i = 0; i < 2520; i = i+42){

    chksm = 0;

    for(int j = 0; j<40; j++){
      chksm = chksm + lidarData[i+j];
    }

    chksm = 0xFF - chksm;

    if(i == 0){
      if(chksm == lidarData[41]){
        good_packet = good_packet + 1;
      }
    } else {
      if(chksm == lidarData[i+40]){
        good_packet = good_packet + 1;
        
      }
    }

  }

  //if received 60 good packets of data from LiDAR, calculate angles etc
  if(good_packet == 60){

    packet_count = packet_count + 1;
    Serial.print("Received good full packet");
    Serial.println(packet_count);

    // now have full data packet, loop over each 42 byte packet
    for(int i = 0; i < 2520; i = i + 42){
      for (int j = i + 6; j < i + 40; j = j + 6) {
        int lidar_distance = (lidarData[j+1] << 8) + lidarData[j];
        int lidar_angle = ((lidarData[i+1] - 160) * 6) + ((j-6-i)/6);

        //Serial.print("Angle = ");
        //Serial.println(lidar_angle);
      }
    }
  }
  
   
  // reset all data to clear for next packet
  good_packet = 0;
  Sync0[0] = 0x00;
  Sync1[0] = 0x00;
  memset(lidarData, 0, sizeof(lidarData));
  memset(remainingData, 0, sizeof(remainingData));

 
}
