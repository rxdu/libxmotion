/*
 *
 *  See the license file included with this source.
 */

#include <iostream>
#include <fstream>
#include <nmeaparse/nmea.h>

using namespace std;
using namespace nmea;

int main(int argc, char **argv)
{
    // --------------------------------------------------------
    // ------------  CONFIGURE GPS SERVICE  -------------------
    // --------------------------------------------------------

    // Create a GPS service that will keep track of the fix data.
    NMEAParser parser;
    GPSService gps(parser);
    //parser.log = true;		// true: will spit out all sorts of parse info on each sentence.

    // Handle events when the lock state changes
    gps.onLockStateChanged += [](bool newlock) {
        if (newlock)
        {
            cout << "\t\t\tGPS aquired LOCK!" << endl;
        }
        else
        {
            cout << "\t\t\tGPS lost lock. Searching..." << endl;
        }
    };

    // Handle any changes to the GPS Fix... This is called after onSentence
    gps.onUpdate += [&gps]() {
        cout << "\t\t\tPosition: " << gps.fix.latitude << "'N, " << gps.fix.longitude << "'E" << endl
             << endl;
    };

    // (optional) - Handle events when the parser receives each sentence
    parser.onSentence += [&gps](const NMEASentence &n) {
        cout << "Received " << (n.checksumOK() ? "good" : "bad") << " GPS Data: " << n.name << endl;
    };

    cout << "-------- Reading GPS NMEA data --------" << endl;

    // --------------------------------------------------------
    // ---------------   STREAM THE DATA  ---------------------
    // --------------------------------------------------------
    try
    {
        // From a buffer in memory...
        //   cout << ">> [ From Buffer]" << endl;
        //   parser.readBuffer((uint8_t*)bytestream, sizeof(bytestream));
        // ---------------------------------------

        // -- OR --
        // From a device byte stream...
        //   cout << ">> [ From Device Stream]" << endl;
        //   parser.readByte(byte_from_device);
        // ---------------------------------------

        // -- OR --
        // From a text log file...
        cout << ">> [ From File]" << endl;
        string line;
        ifstream file("nmea_log.txt");
        while (getline(file, line))
        {
            try
            {
                parser.readLine(line);
            }
            catch (NMEAParseError &e)
            {
                cout << e.message << endl
                     << endl;
                // You can keep feeding data to the gps service...
                // The previous data is ignored and the parser is reset.
            }
        }
    }
    catch (exception &e)
    {
        // Notify the proper authorities. Something is on fire.
        cout << "Something Broke: " << e.what() << endl;
    }
    // ---------------------------------------

    // Show the final fix information
    //cout << gps.fix.toString() << endl;

    cout << "-------- ALL DONE --------" << endl;

    cin.ignore();

    return 0;
}