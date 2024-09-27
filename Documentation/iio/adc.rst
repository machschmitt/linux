.. SPDX-License-Identifier: GPL-2.0

=========================
IIO Abstractions for ADCs
=========================

1. Overview
===========

The IIO subsystem supports many Analog to Digital Converters (ADCs). Some ADCs
have features and characteristics that have been supported in peculiar ways by
IIO device drivers. This documentation describes ADC specific features and
explains how they are (should be?) supported by the IIO subsystem.

1. ADC Channel Types
====================

ADCs can have distinct types of inputs, each of them measuring analog voltages
in a slightly different way. An ADC digitizes the analog input voltage over a
span given by the provided voltage reference, the input type, and the input
polarity. The input range allowed to an ADC channel is needed to determine the
scale factor and offset for the channel. The scale and offset attributes are
then read by user space applications to obtain the measured value in real-world
units (millivolts for voltage measurement, milliamps for current measurement,
etc.).

There are three types of ADC channels.

1.1 Single-ended channels
-------------------------

1.1.1 Single-ended Unipolar Channels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

  ---------- VREF -------------
      ´ `           ´ `                  _____________
    /     \       /     \               /             |
   /       \     /       \         --- <  IN    ADC   |
            \   /         \   /         \             |
             `-´           `-´           \  GND  VREF |
  -------- GND (0V) -----------           +-----------+
                                                  ^
                                                  |
                                             External VREF

Legend::

  Single-ended Unipolar Channel

For **single-ended unipolar** channels, the analog voltage input can swing from
0V to VREF (where VREF is a voltage reference with voltage potential higher than
system ground (GND)). The maximum input voltage is often called VFS (full-scale
input voltage), with VFS being determined by VREF. The voltage reference may be
provided from an external supply or derived from the chip power source.

1.1.2 Single-ended Bipolar Channels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

  ---------- +VREF ------------
      ´ `           ´ `                  _____________________
    /     \       /     \               /                     |
   /       \     /       \         --- <  IN          ADC     |
            \   /         \   /         \                     |
             `-´           `-´           \  GND  +VREF  -VREF |
  ---------- -VREF ------------           +-------------------+
                                                  ^       ^
                                                  |       +---- External -VREF
                                           External +VREF

Legend::

  Single-ended Bipolar Channel

The input voltage to a **single-ended bipolar** channel may go from -VREF to
+VREF (where -VREF is the voltage reference that has the lower electrical
potential while +VREF is the reference with the higher one). The ADC chip may
derive the lower reference from +VREF or get it from a separate input. Often,
+VREF and -VREF are symmetric but they don't need to be so. When -VREF is lower
than system ground, these inputs are also called single-ended true bipolar.

1.2 Differential channels
-------------------------

1.2.1 Differential Bipolar Channels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

  -------- +VREF ------
    ´ `       ´ `               +-------------------+
  /     \   /     \   /        /                    |
         `-´       `-´    --- <  IN+                |
  -------- -VREF ------        |                    |
                               |            ADC     |
  -------- +VREF ------        |                    |
        ´ `       ´ `     --- <  IN-                |
  \   /     \   /     \        \       +VREF  -VREF |
   `-´       `-´                +-------------------+
  -------- -VREF ------                  ^       ^
                                         |       +---- External -VREF
                                  External +VREF

Legend::

  Differential Bipolar Channel

The analog signals to **differential bipolar** inputs are also allowed to swing
from -VREF to +VREF. However, a differential voltage measurement digitizes the
voltage level at the positive input (IN+) relative to the negative input (IN-)
over the -VREF to +VREF span. In other words, a differential channel measures
how many volts IN+ is away from IN- (IN+ - IN-). If -VREF is below system GND,
these are also called differential true bipolar inputs.

1.2.2 Differential Unipolar Channels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For **differential unipolar** channels, the analog voltage at the positive input
must also stay above the voltage level at the negative input. Thus, the actual
input range allowed to a differential unipolar channel is IN- to +VREF. Because
IN+ is allowed to swing with the measured analog signal and the input setup must
guarantee IN+ will not go below IN- (nor IN- will raise above IN+), most
differential unipolar channel setups have IN- fixed to a known voltage that does
not fall within the voltage range expected for the measured signal. This leads
to a setup that is equivalent to a pseudo-differential channel. Thus,
differential unipolar channels are actually pseudo-differential unipolar
channels.

1.3 Pseudo-differential Channels
--------------------------------

There is a third ADC input type which is called pseudo-differential or
single-ended to differential configuration. A pseudo-differential input is made
out from a differential pair of inputs by restricting the negative input to a
known voltage while allowing only the positive input to change.

1.3.1 Pseudo-differential Unipolar Channels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

  -------- +VREF ------
    ´ `       ´ `                +-------------------+
  /     \   /     \   /         /                    |
         `-´       `-´     --- <  IN+                |
  --------- IN- -------         |                    |
                                |            ADC     |
                                |                    |
  Common-mode voltage -->  --- <  IN-                |
                                \       +VREF  -VREF |
                                 +-------------------+
                                          ^       ^
                                          |       +---- External -VREF
                                   External +VREF

Legend::

  Pseudo-differential Unipolar Channel

A **pseudo-differential unipolar** input has the limitations a differential
unipolar channel would have, meaning the analog voltage to the positive input
IN+ must stay within IN- to +VREF. The fixed voltage to IN- is sometimes called
common-mode voltage and it must be within -VREF to +VREF as would be expected
from the signal to any differential channel negative input.

In pseudo-differential configuration, the voltage measured from IN+ is not
relative to GND (as it would be for a single-ended channel) but to IN-, which
causes the measurement to always be offset by IN- volts. To allow applications
to calculate IN+ voltage with respect to system ground, the IIO channel may
provide an _offset attribute to report the channel offset to user space.

1.3.2 Pseudo-differential Bipolar Channels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

  -------- +VREF ------
    ´ `       ´ `                +-------------------+
  /     \   /     \   /         /                    |
         `-´       `-´     --- <  IN+                |
  -------- -VREF ------         |                    |
                                |            ADC     |
                                |                    |
  Common-mode voltage -->  --- <  IN-                |
                                \       +VREF  -VREF |
                                 +-------------------+
                                          ^       ^
                                          |       +---- External -VREF
                                   External +VREF

Legend::

  Pseudo-differential Bipolar Channel

A **pseudo-differential bipolar** input is not limited by the level at IN- but
it will be limited to -VREF or to GND on the lower end of the input range
depending on the particular ADC. Similar to their unipolar counter parts,
pseudo-differential bipolar channels may define an _offset attribute to provide
the read offset relative to GND.

In some setups, the analog signal passes though an amplifier or gain
circuitry before reaching the ADC inputs. In those cases,
the actual input range is smaller (if the signal is amplified (gain > 1))
or larger (if the signal is attenuated (gain < 1)) than the input
range for each input type and polarity discussed above. To account
for that, the input range is divided (or multiplied) by the gain
factor.

In many cases, the negative reference (-VREF) is 0V (GND), but it may
be higher than GND (e.g. 2.5V) or even lower (e.g. -2.5V).
Regardless of the provided voltage reference(s), the analog inputs
must stay within 0V to VREF (for single-ended inputs) or within -VREF to
+VREF (for differential inputs).
With that, the least significant bit (LSB) of the ADC output code
depends on the input range and, for simple ADCs that output data
conversion in straight binary format, the LSB can be calculated as
input_range / 2^(precision_bits).
For example, if the device has 16-bit precision, VREF = 5V, and the
input is single-ended unipolar, then one LSB will represent
(VREF - 0V)/2^16 = 0.000076293945 V or 76.293945 micro volts.
If the input is differential bipolar, -VREF = 2.5V, and +VREF = 5V, then
1 LSB = (+VREF - (-VREF))/2^16 = 2.5/2^16 = 38.146973 micro volts.
