% Some necessary initial conditions for Simulink to work. These are
% arbitrary but evidently necessary.
Busses;

rudderCommandIC = Simulink.Bus.createMATLABStruct('RudderCommand');
rudderCommandIC.enable = false;
rudderCommandIC.direction = true;
rudderCommandIC.up = uint16(0);
rudderCommandIC.period = uint16(0);

throttleCommandIC = Simulink.Bus.createMATLABStruct('ThrottleCommand');
throttleCommandIC.identifier = uint32(0);
throttleCommandIC.data = uint8([0 0 0 0 0 0]);
throttleCommandIC.size = uint8(0);
throttleCommandIC.trigger = false;