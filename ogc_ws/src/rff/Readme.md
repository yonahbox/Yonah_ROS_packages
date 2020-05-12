# RFF

## Button

### Files

The `RFF` node is in the `scripts/` directory

## Working

The node connects to the router via ssh and continuously checks the state of the button.

It will publish to `/buttonpress` once the button is pressed. The button will be set to blink 5 times if the button is held for more than 2 seconds. 

# License
This package is licensed under the GNU GPL version 3 license or later. Refer to the COPYING.txt file in the root folder of this repository for more details
