/*
   PrintFile.ino : Script that sends via serial the contents of
   the file datalog.csv stored in BonaDrone's FC Flash.

   For the script used to collect data see: 

   	   https://github.com/BonaDrone/ESKF/blob/master/extras/arduino/CollectData/CollectData.ino

   Hardware support for Bonadrone flight controller:

       https://github.com/BonaDrone/grumpyoldpizza

   Copyright (c) 2019 Juan Gallostra
 */

#include <FS.h>

File myFile;

void setup(void)
{

	Serial.begin(9600);

	DOSFS.begin();
	
	delay(5000);

	myFile = DOSFS.open("datalog.csv", "r");
	if (myFile)
	{
		while (myFile.available())
		{
			Serial.write(myFile.read());
			delay(5);
		}

		myFile.close();
	}
	else
	{
		Serial.println("Error opening datalog.csv");
	}

}

void loop(void)
{

}
