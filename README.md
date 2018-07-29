# HomeEasy-HE100-Analysis

Having searched lots of repositories for decoding the HomeEasy protocol, I couldn't find any that deciphered the HE100 correctly. People seem to have mixed ideas on what various parts of the data are defined as.

I started by using an Saleae USB logic analyser to collect the signal pulses coming from a cheap 433MHz receiver. I then created code, to decipher that signal piece by piece.

The original code was based on someone else's code that processed the incoming signal based on a hardware tigger for GPIO state change, but I found that lacking and most of the original code is gone.

I am not sure if this following 'feature' is unique to my HE100, or whether all HE100's share it, or whether it is the hardware I am using to capture the signal, but a '0' bit is a high 310 uS followed by a 220uS low instead of two equal 310uS lengths. When I tried to verify this with rtl_433, the debug showed no time difference between the states - though, I may have not been using rtl_433 correctly.
