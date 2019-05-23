/* DOSFS read/write
 *
 * This example shows how to read and write data to and from a DOSFS file
 *
 * Code adopted from the SD Library
 *    
 * This example code is in the public domain.
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
		Serial.println("Reading from datalog.csv... ");

		while (myFile.available())
		{
			Serial.write(myFile.read());
		}

		myFile.close();
	}
	else
	{
		Serial.println("Error opening datalog.csv");
	}

	Serial.println("Done");
}

void loop(void)
{

}
