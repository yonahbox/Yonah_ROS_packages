# Telegram link

## Files

### Launch files
Currently there is only one launch file `telegram.launch` that has to be run

#### Argurments

tdlib_auth_dir: location of the telegram client instance
monitored_chat: Name of the telegram chat (or group) the node will listen to/send messages to. Currently only supports one chat
command_whitelist: List of users the node will listen to. Currently requires the users first name

### Node files

The `telegram_link` node is in the `src/` directory

### Other files

The `scripts/` directory contains `Td.py` which is contains all the telegram specific functions

## Working

The node listens to `ogc/to_telegram` and outputs all the received messages from that topic into the telegram chat specified in the launch file.

Any messages received from the telegram chat is published onto the `ogc/from_telegram` topic


# License
This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details
