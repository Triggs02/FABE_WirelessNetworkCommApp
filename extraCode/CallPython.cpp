#include <cstdlib>
#include <iostream>
#include <fstream>
#include <random>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <filesystem> 

int main() {
    // Command to execute the Python script with parameter 7
    const char* command = "python3 ~/github/Repos/ECE3905-Team4-PugComms/stats.py 4";
    
    // Execute the command
    int status = system(command);

    // Check the status of the execution
    if (status == -1) {
        // System call failed
        return 1;
    } else {
        // Check the exit status of the Python script
        if (WIFEXITED(status)) {
            int exit_status = WEXITSTATUS(status);
            std::cout << "Script exited with status " << exit_status << std::endl;
        } else {
            std::cout << "Script did not exit normally" << std::endl;
        }
    }

    return 0;
}
