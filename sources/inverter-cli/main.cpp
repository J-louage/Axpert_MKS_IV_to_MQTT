// Lightweight program to take the sensor data from a Voltronic Axpert, Mppsolar PIP, Voltacon, Effekta, and other branded OEM Inverters and send it to a MQTT server for ingestion...
// Adapted from "Maio's" C application here: https://skyboo.net/2017/03/monitoring-voltronic-power-axpert-mex-inverter-under-linux/
//
// Please feel free to adapt this code and add more parameters -- See the following forum for a breakdown on the RS323 protocol: http://forums.aeva.asn.au/viewtopic.php?t=4332
// ------------------------------------------------------------------------

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

#include "main.h"
#include "tools.h"
#include "inputparser.h"

#include <pthread.h>
#include <signal.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>


bool debugFlag = false;
bool runOnce = false;

cInverter *ups = NULL;

atomic_bool ups_status_changed(false);
atomic_bool ups_qmod_changed(false);
atomic_bool ups_qpiri_changed(false);
atomic_bool ups_qpigs_changed(false);
atomic_bool ups_qpiws_changed(false);
atomic_bool ups_cmd_executed(false);


// ---------------------------------------
// Global configs read from 'inverter.conf'

string devicename;
int runinterval;
float ampfactor;
float wattfactor;
int qpiri, qpiws, qmod, qpigs;

// ---------------------------------------

void attemptAddSetting(int *addTo, string addFrom) {
    try {
        *addTo = stof(addFrom);
    } catch (exception e) {
        cout << e.what() << '\n';
        cout << "There's probably a string in the settings file where an int should be.\n";
    }
}

void attemptAddSetting(float *addTo, string addFrom) {
    try {
        *addTo = stof(addFrom);
    } catch (exception e) {
        cout << e.what() << '\n';
        cout << "There's probably a string in the settings file where a floating point should be.\n";
    }
}

void getSettingsFile(string filename) {

    try {
        string fileline, linepart1, linepart2;
        ifstream infile;
        infile.open(filename);

        while(!infile.eof()) {
            getline(infile, fileline);
            size_t firstpos = fileline.find("#");

            if(firstpos != 0 && fileline.length() != 0) {    // Ignore lines starting with # (comment lines)
                size_t delimiter = fileline.find("=");
                linepart1 = fileline.substr(0, delimiter);
                linepart2 = fileline.substr(delimiter+1, string::npos - delimiter);

                if(linepart1 == "device")
                    devicename = linepart2;
                else if(linepart1 == "run_interval")
                    attemptAddSetting(&runinterval, linepart2);
                else if(linepart1 == "amperage_factor")
                    attemptAddSetting(&ampfactor, linepart2);
                else if(linepart1 == "watt_factor")
                    attemptAddSetting(&wattfactor, linepart2);
                else if(linepart1 == "watt_factor")
                    attemptAddSetting(&wattfactor, linepart2);
                else if(linepart1 == "qpiri")
                    attemptAddSetting(&qpiri, linepart2);
                else if(linepart1 == "qpiws")
                    attemptAddSetting(&qpiws, linepart2);
                else if(linepart1 == "qmod")
                    attemptAddSetting(&qmod, linepart2);
                else if(linepart1 == "qpigs")
                    attemptAddSetting(&qpigs, linepart2);
                else
                    continue;
            }
        }
        infile.close();
    } catch (...) {
        cout << "Settings could not be read properly...\n";
    }
}

int main(int argc, char* argv[]) {

    // Reply1
    float voltage_grid;
    float freq_grid;
    float voltage_out;
    float freq_out;
    int load_va;
    int load_watt;
    int load_percent;
    int voltage_bus;
    float voltage_batt;
    int batt_charge_current;
    int batt_capacity;
    int temp_heatsink;
    float pv_input_current;
    float pv_input_voltage;
    float pv_input_watts;
    float pv_input_watthour;
    float load_watthour = 0;
    float scc_voltage;
    int batt_discharge_current;
    char device_status[9];

    // Reply2
    float grid_voltage_rating;
    float grid_current_rating;
    float out_voltage_rating;
    float out_freq_rating;
    float out_current_rating;
    int out_va_rating;
    int out_watt_rating;
    float batt_rating;
    float batt_recharge_voltage;
    float batt_under_voltage;
    float batt_bulk_voltage;
    float batt_float_voltage;
    int batt_type;
    int max_grid_charge_current;
    int max_charge_current;
    int in_voltage_range;
    int out_source_priority;
    int charger_source_priority;
    int machine_type;
    int topology;
    int out_mode;
    float batt_redischarge_voltage;

    int mask_b;
    int mask_c;
    int parallel_max_num;

    // Get command flag settings from the arguments (if any)
    InputParser cmdArgs(argc, argv);
    const string &rawcmd = cmdArgs.getCmdOption("-r");

    if(cmdArgs.cmdOptionExists("-h") || cmdArgs.cmdOptionExists("--help")) {
        return print_help();
    }
    if(cmdArgs.cmdOptionExists("-d")) {
        debugFlag = true;
    }
    if(cmdArgs.cmdOptionExists("-1") || cmdArgs.cmdOptionExists("--run-once")) {
        runOnce = true;
    }
    lprintf("INVERTER: Debug set");

    // Get the rest of the settings from the conf file
    if( access( "./inverter.conf", F_OK ) != -1 ) { // file exists
        getSettingsFile("./inverter.conf");
    } else { // file doesn't exist
        getSettingsFile("/etc/inverter/inverter.conf");
    }

    bool ups_status_changed(false);
    ups = new cInverter(devicename,qpiri,qpiws,qmod,qpigs);

    // Logic to send 'raw commands' to the inverter..
    if (!rawcmd.empty()) {
        ups->ExecuteCmd(rawcmd);
        // We're piggybacking off the qpri status response...
        printf("Reply:  %s\n", ups->GetQpiriStatus()->c_str());
        exit(0);
    } else {
        ups->runMultiThread();
    }

    while (true) {
        if (ups_status_changed) {
            int mode = ups->GetMode();

            if (mode)
                lprintf("INVERTER: Mode Currently set to: %d", mode);

            ups_status_changed = false;
        }

        if (ups_qmod_changed && ups_qpiri_changed && ups_qpigs_changed) {

            ups_qmod_changed = false;
            ups_qpiri_changed = false;
            ups_qpigs_changed = false;

            int mode = ups->GetMode();
            string *reply1   = ups->GetQpigsStatus();
            string *reply2   = ups->GetQpiriStatus();
            string *warnings = ups->GetWarnings();

            if (reply1 && reply2 && warnings) {

		//QPIGS (Current Values)
		//
                // Parse and display values
		//0 Grid Volts: 209.7 -float
		//1 Grid Frequency: 50.0 -float
		//2 AC Output Volts: 230.1 -float
		//3 AC Output Frequency: 49.9 -float
		//4 AC Output Apparent Power: 0667 -float
		//5 AC Output Active Power: 0550 -float
		//6 Output Load Percent: 011 -float
		//7 Bus Voltage: 361 -float
		//8 Battery Voltage: 52.20 -float
		//9 Battery Charging Current: 000 -float
		//10 Battery Capacity: 061 -float
		//11 Heat Sink Temperature: 0031 -float
		//12 Solar Input Current: 00.0 -float
		//13 Solar Input Voltage: 000.0 -float
		//14 Battery voltage from scc: 00.00 -float
		//15 Battery discharge current: 00011 -float
		//16 ---Reserved U7, U6: 00 [0] [1]
		//17 Scc Firmware Update: 0 //0 No - 1 Yes [2]
		//18 Load On: 1 [3]
		//19 Reserved U4: 0 [4]
		//20 Charging Status: 000 //000 Not Charging - 101 AC Charging - 110 Solar Charging - 111 Solar and AC Charging [5][6][7]

		// --16 device status --17 mask_b --18 mask_c --19 PV Active Power

		//21 ---Reserved Y: 00
		//22 ---Reserved Z: 00
		//23 ---Reserved AA: 00000
		//24 ---Reserved BB: 010


		//QPIRI (Configuration Values)
		//----------------------------
		//0 grid rating voltage: 230.0
		//1 grid rating current: 24.3
		//2 ac output rating voltage: 230.0
		//3 ac output rating frequency: 50.0
		//4 ac output rating current: 24.3
		//5 ac output rating apparent power: 5600
		//6 ac output rating active power: 5600
		//7 battery rating voltage: 48.0
		//8 battery recharge voltage: 46.0
		//9 battery under voltage: 44.8
		//10 battery bulk voltage: 57.6
		//11 battery float voltage: 57.6
		//12 battery type: 3
		//13 current max ac charging: 030
		//14 current max charging current: 100
		//15 input voltage range: 0
		//16 output source priority: 2
		//17 charger source priority: 2

		//18 parallel max num: 9 -------

		//19 machine type: 00
		//20 topology: 0
		//21 output mode: 0
		//22 battery re discharge voltage: 52.0

		//23 pv condition: 0
		//24 pv power balance: 1
		//000

		//reply1: 000.0 00.0 230.0 50.0 0414 0372 007 399 53.50 019 048 0041 06.4 244.4 00.00 00000 00010010 00 00 01556 010
		//reply2: 230.0 24.3 230.0 50.0 24.3 5600 5600 48.0 46.0 44.8 57.6 57.6 3 030 100 0 2 2 9 00 0 0 52.0 0 1 000

                sscanf(reply1->c_str(), "%f %f %f %f %d %d %d %d %f %d %d %d %f %f %f %d %s %d %d %f", &voltage_grid, &freq_grid, &voltage_out, &freq_out, &load_va, &load_watt, &load_percent, &voltage_bus, &voltage_batt, &batt_charge_current, &batt_capacity, &temp_heatsink, &pv_input_current, &pv_input_voltage, &scc_voltage, &batt_discharge_current, &device_status, &mask_b, &mask_c, &pv_input_watts);
                sscanf(reply2->c_str(), "%f %f %f %f %f %d %d %f %f %f %f %f %d %d %d %d %d %d %d %d %d %d %f", &grid_voltage_rating, &grid_current_rating, &out_voltage_rating, &out_freq_rating, &out_current_rating, &out_va_rating, &out_watt_rating, &batt_rating, &batt_recharge_voltage, &batt_under_voltage, &batt_bulk_voltage, &batt_float_voltage, &batt_type, &max_grid_charge_current, &max_charge_current, &in_voltage_range, &out_source_priority, &charger_source_priority, &parallel_max_num, &machine_type, &topology, &out_mode, &batt_redischarge_voltage);

                // There appears to be a discrepancy in actual DMM measured current vs what the meter is
                // telling me it's getting, so lets add a variable we can multiply/divide by to adjust if
                // needed.  This should be set in the config so it can be changed without program recompile.
                if (debugFlag) {
                    printf("INVERTER: ampfactor from config is %.2f\n", ampfactor);
                    printf("INVERTER: wattfactor from config is %.2f\n", wattfactor);
		    printf("Start REply\n");
		    printf("reply1: %s\n", reply1->c_str());
		    printf("reply2: %s\n", reply2->c_str());
		    printf("End Reply\n");
                }

                pv_input_current = pv_input_current * ampfactor;

                // It appears on further inspection of the documentation, that the input current is actually
                // current that is going out to the battery at battery voltage (NOT at PV voltage).  This
                // would explain the larger discrepancy we saw before.

                //pv_input_watts = (scc_voltage * pv_input_current) * wattfactor;

                // Calculate watt-hours generated per run interval period (given as program argument)
                pv_input_watthour = pv_input_watts / (3600 / runinterval);
                load_watthour = (float)load_watt / (3600 / runinterval);

                // Print as JSON (output is expected to be parsed by another tool...)
                printf("{\n");

                printf("  \"Inverter_mode\":%d,\n", mode);
                printf("  \"AC_grid_voltage\":%.1f,\n", voltage_grid);
                printf("  \"AC_grid_frequency\":%.1f,\n", freq_grid);
                printf("  \"AC_out_voltage\":%.1f,\n", voltage_out);
                printf("  \"AC_out_frequency\":%.1f,\n", freq_out);
                printf("  \"PV_in_voltage\":%.1f,\n", pv_input_voltage);
                printf("  \"PV_in_current\":%.1f,\n", pv_input_current);
                printf("  \"PV_in_watts\":%.1f,\n", pv_input_watts);
                printf("  \"PV_in_watthour\":%.4f,\n", pv_input_watthour);
                printf("  \"SCC_voltage\":%.4f,\n", scc_voltage);
                printf("  \"Load_pct\":%d,\n", load_percent);
                printf("  \"Load_watt\":%d,\n", load_watt);
                printf("  \"Load_watthour\":%.4f,\n", load_watthour);
                printf("  \"Load_va\":%d,\n", load_va);
                printf("  \"Bus_voltage\":%d,\n", voltage_bus);
                printf("  \"Heatsink_temperature\":%d,\n", temp_heatsink);
                printf("  \"Battery_capacity\":%d,\n", batt_capacity);
                printf("  \"Battery_voltage\":%.2f,\n", voltage_batt);
                printf("  \"Battery_charge_current\":%d,\n", batt_charge_current);
                printf("  \"Battery_discharge_current\":%d,\n", batt_discharge_current);
                printf("  \"Load_status_on\":%c,\n", device_status[3]);
		//New
		printf("  \"Battery_charging\":%c,\n", device_status[5]);

		printf("  \"mask_b\":%d,\n", mask_b);
		printf("  \"mask_c\":%d,\n", mask_c);
		printf("  \"parallel_max_num\":%d,\n", parallel_max_num);

                printf("  \"SCC_charge_on\":%c,\n", device_status[6]);
                printf("  \"AC_charge_on\":%c,\n", device_status[7]);
                printf("  \"Battery_recharge_voltage\":%.1f,\n", batt_recharge_voltage);
                printf("  \"Battery_under_voltage\":%.1f,\n", batt_under_voltage);
                printf("  \"Battery_bulk_voltage\":%.1f,\n", batt_bulk_voltage);
                printf("  \"Battery_float_voltage\":%.1f,\n", batt_float_voltage);
                printf("  \"Max_grid_charge_current\":%d,\n", max_grid_charge_current);
                printf("  \"Max_charge_current\":%d,\n", max_charge_current);
                printf("  \"Out_source_priority\":%d,\n", out_source_priority);
                printf("  \"Charger_source_priority\":%d,\n", charger_source_priority);
                printf("  \"Battery_redischarge_voltage\":%.1f,\n", batt_redischarge_voltage);
                printf("  \"Warnings\":\"%s\"\n", warnings->c_str());
                printf("}\n");

                // Delete reply string so we can update with new data when polled again...
                delete reply1;
                delete reply2;

                if(runOnce) {
                    // Do once and exit instead of loop endlessly
                    lprintf("INVERTER: All queries complete, exiting loop.");
                    exit(0);
                }
            }
        }

        sleep(1);
    }

    if (ups)
        delete ups;
    return 0;
}
