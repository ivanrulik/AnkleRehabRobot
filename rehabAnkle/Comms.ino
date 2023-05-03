const byte numChars = 64;
char receivedChars[numChars];

boolean newData = false;

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}
void parseData()
{
   char *strings[4]; // an array of pointers to the pieces of the above array after strtok()
   char *ptr = NULL; byte index = 0;
   ptr = strtok(receivedChars, ",");  // delimiters, comma
   while (ptr != NULL)
   {
      strings[index] = ptr;
      index++;
      ptr = strtok(NULL, ",");
   }
   // convert string data to numbers
   if(newData == true)
   {
     tPos1 = atof(strings[0]);
     tPos2 = atof(strings[1]);
     tPos3 = atof(strings[2]);
//     Serial.print(tPos1);
//     Serial.print(" ");
//     Serial.print(tPos2);
//     Serial.print(" ");
//     Serial.println(tPos3);
   }
   newData = false;
}

void replyToPython() {
    if (newData == true) {
        Serial.print("<This just in ... ");
        Serial.print(receivedChars);
        Serial.print("   ");
        Serial.print(millis());
        Serial.print('>');
        newData = false;
    }
}
