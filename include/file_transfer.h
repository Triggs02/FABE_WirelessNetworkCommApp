// file_transfer.h

#ifndef FILE_TRANSFER_H
#define FILE_TRANSFER_H

#include <iostream>
#include <string>
#include <cstdlib> // For system()

class FileTransfer {
public:
    static void transferFile(const std::string& fileName, const std::string& userName, const std::string& password, const std::string& ipAddress, const std::string& fileDestination) {
        // Construct the SCP command using sshpass
        std::string command = "sshpass -p '" + password + "' scp " + fileName + " " + userName + "@" + ipAddress + ":/C:/Users/boena/Desktop/TestFolder/" ;

        // Execute the command
        int result = system(command.c_str());

        // Check the result
        if (result != 0) {
            std::cerr << "Failed to transfer file. Command exited with code " << result << std::endl;
        }
    }
};

#endif // FILE_TRANSFER_H