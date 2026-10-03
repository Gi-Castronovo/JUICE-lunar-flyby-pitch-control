function [value, isterminal, direction] = saturation_event(~, s, w_limit)
%SATURATION_EVENT Stops the integration when |w_wheel| reaches w_limit.

value      = abs(s(7)) - w_limit;
isterminal = 1;
direction  = 1;
end
