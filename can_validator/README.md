*Copied from MIT EVT's Can_Validator library*

I just finished up a little tool that will hopefully be useful managing traffic on the vehicle's CAN bus. Sometime on Saturday or otherwise, add the specs for the CAN messages that your module needs to use here.

The format* should be pretty self explanatory (but feel free to ask questions). Then run:

    ./check.py --validate_spec example_can_spec.txt

to make sure than the changes you made are A+, consistent, and valid*. Down the road, we can run MiniMon to listen over UART and produce a CSV log of all CAN messages on the bus.

    ./check.py --check_log example_can_spec.txt example_minimon_log.csv

will validate the spec, produce a summarized log output, and check that what we see in the log matches the spec-defined bus behavior. Code and examples found here.

---------------------------

* What is actually being checked when you 'validate' the spec? A few things:

- Messages don't have repeated symbolic names or conflicting IDs
- Defined data fields do not overlap and fit within the CAN frame data field
- Data field values and symbols are 1:1 (one data value doesn't map to two symbols)
- Specified CAN ID fits within the CAN frame data field
- And a few other sanitary things

** What is actually being done/checked/produced when I compare the log against the defined spec?

- Frequency constraints are met (e.g. is the heartbeat being sent out atleast every 1 second?)
- Consecutive, repeated messages are compressed into 'message blocks' with start and end times. This dramatically improves readability of the log
- IDs and data fields are converted to their human-readable symbols (as defined in the spec!)


*There are a few features that the spec validator accepts, but aren't in the example spec file (e.g. range of data thresholds, float data values, etc.). A full demonstration of the format can now be found in `opel_gt_can_spec.txt`.
