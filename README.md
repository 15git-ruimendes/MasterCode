# MasterCode

The code is arranged in a way such that,
the sensitization interrupts and accompanying function,
the safety state machines,
file handler structure and set up and system functions,
and the control schemas get their own library and corresponding interrupt functions.

## 0 - Card Handler

Within this library lie the functions and structures to start, stop and recall values from all of the system cards.

The start or setup function should be called at the start of each code run such that all of the cards are ready and capable to stop and reset the system whenever SOA is surpassed.

## 1 - File Handler

Within this library lie the functions and structures to start, stop and recall values from all of the system cards.

The start or setup function should be called at the start of each code run such that all of the cards are ready and capable to stop and reset the system whenever SOA is surpassed.

## 2 - Safety Handler

This is a critical function containing the state machines that control pre-charge, over-temperature, over-load and other safety critical values. The turn on, turn off and recall functions as a connection to the HM interface.

## 3 - Sensor Reading

This library must contain all filters, transforms and estimation functions needed for the control of the converter.

## 4 - Control Schema

This library contains the previously created structures and functions to perform P,Pi,PID and PR controllers as well as other control or modulation directed functions.

## Master Plan

This library arranges and controls the interrupts that call other structural functions according to system status.

![alt text](https://media.makeameme.org/created/my-master-plan-593558.jpg)
![alt text](https://preview.redd.it/t60h2airs1o61.png?width=960&crop=smart&auto=webp&v=enabled&s=a02572042a3a1ebb45408e06feb130c7f3b14702)
