#pragma once

#ifndef NETWORK_CREDENTIALS_H
#define NETWORK_CREDENTIALS_H

// Defining important network credentials that are used in network communication
// b/w the RPi and server destination for all transmitted data
// NOTE: Make sure to supply valid credentials for server that has OpenSSH enabled
//       and will be used as the destination for collected data.
namespace networkCreds {
    // Login credentials for server
    const char *userName = "Your_UserName_Here";
    const char *password = "Your_Password_Here";

    // IP-Address
    const char *serverIPAddr = "Server_IP_Addr_Here";


    const char *fileTransferPath = "/File/Transfer/Path/Here";
}
#endif