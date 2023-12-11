%Model of additional load
%%Description: simulate disturbance as additional load on the object with costant gradient
%%Inputs:      time = current simulation time [s]
%              t_dist = instant of distrubance addition [s] 
%Output:       f_add = additional weight function as load on z-axis with constant gradient [N]


function f_add = add_load(time,t_dist)
    add_weight = 5; %N
    m = add_weight/0.3; %angular coefficient
    f_add = 0 + m*(time - t_dist);
    if f_add >= add_weight
        f_add = add_weight;
    end
end
   