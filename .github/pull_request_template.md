Closes #

## What changed, and why

<!--
Two or three sentences. What was the behaviour before, and what is it now? If there is no issue to close, say here what prompted the change.
-->

## Type

- [ ] Bug fix
- [ ] New feature
- [ ] Tuning or constants
- [ ] Refactor, no behaviour change
- [ ] Autos or paths
- [ ] Cleanup or docs
- Other: 

## What Systems does this affect

<!-- 
Mention every subsystem this pull request changes the behaviour of. 
e.g. drive, climber, and controller bindings
-->

## Hardware this assumes

<!--
Delete this section if the change is pure software.

- CAN ids, or a device that has to be on a particular bus
- Current limits
- Gear ratios or `SensorToMechanismRatio`
- Inverted, or neutral mode
- Anything that has to be re-zeroed, re-flashed or re-configured in Tuner
-->

## How this was tested

<!-- Tick what you actually did. Do not tick ahead. -->

- [ ] Builds locally
- [ ] Ran in simulation
- [ ] Ran on the test chassis or a bench motor
- [ ] Ran on the full robot in the shop
- [ ] Ran during driver practice
- [ ] Ran at an event
- [ ] Not tested on hardware yet, **explained in description**
- Other

**Additional Evidence:**

<!--
A video, a screenshot, or an AdvantageScope plot. Drag it in, or link the Slack message if needed. 

If this cannot be tested until the robot is together, say so and say what you expect to happen when it is.
-->

## What could this break

<!--
Say what else depends on what you changed, and what you checked. "Nothing, this constant is only read by the indexer" is a fine answer.
-->

## Before merging

- [ ] `WPILib: Build Robot Code` says `BUILD SUCCESSFUL`
- [ ] I built at least once after my last edit, so the formatter check will pass
- [ ] I read my own diff before asking anyone else to
- [ ] Commented-out code is either deleted or has a comment saying why it stays
