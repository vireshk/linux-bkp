Linux Kernel GPIO based logic analyzer
======================================

:Author: Wolfram Sang

Introduction
------------

This document briefly describes how to run the software based in-kernel logic
analyzer.

Note that this is still a last resort analyzer which can be affected by
latencies and non-determinant code paths. However, for e.g. remote development,
it may be useful to get a first view and aid further debugging.

Setup
-----

Tell the kernel which GPIOs are used as probes. For a DT based system:

    i2c-analyzer {
            compatible = "gpio-logic-analyzer";
            probe-gpios = <&gpio6 21 GPIO_OPEN_DRAIN>, <&gpio6 4 GPIO_OPEN_DRAIN>;
            probe-names = "SCL", "SDA";
    };

The binding documentation is in the ``misc`` folder of the Kernel binding
documentation.

Usage
-----

The logic analyzer is configurable via files in debugfs. However, it is
strongly recommended to not use them directly, but to to use the
``gpio-logic-analyzer`` script in the ``tools/debugging`` directory. Besides
checking parameters more extensively, it will isolate a CPU core for you, so
you will have least disturbance while measuring.

The script has a help option explaining the parameters. For the above DT
snipplet which analyzes an I2C bus at 400KHz on a Renesas Salvator-XS board,
the following settings are used: The isolated CPU shall be CPU1 because it is a
big core in a big.LITTLE setup. Because CPU1 is the default, we don't need a
parameter. The bus speed is 400kHz. So, the sampling theorem says we need to
sample at least at 800kHz. However, falling of both, SDA and SCL, in a start
condition is faster, so we need a higher sampling frequency, e.g. ``-s
1500000`` for 1.5MHz. Also, we don't want to sample right away but wait for a
start condition on an idle bus. So, we need to set a trigger to a falling edge
on SDA, i.e. ``-t "2F"``. Last is the duration, let us assume 15ms here which
results in the parameter ``-d 15000``. So, altogether:

    gpio-logic-analyzer -s 1500000 -t "2F" -d 15000

Note that the process will return you back to the prompt but a sub-process is
still sampling in the background. Unless this finished, you will not find a
result file in the current or specified directory. Please also note that
currently this sub-process is not killable! For the above example, we will then
need to trigger I2C communication:

    i2cdetect -y -r <your bus number>

Result is a .sr file to be consumed with PulseView from the free Sigrok project. It is
a zip file which also contains the binary sample data which may be consumed by others.
The filename is the logic analyzer instance name plus a since-epoch timestamp.
