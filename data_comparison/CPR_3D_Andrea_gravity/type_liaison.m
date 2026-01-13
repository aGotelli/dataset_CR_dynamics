function [A,bar_A] = type_liaison(type)

if type == 'R'
    A = [0;0;1;0;0;0];
    bar_A = [[eye(2,2);zeros(3,2)],zeros(5,1),[zeros(2,3);eye(3,3)]];
elseif type == 'S'
    A = [eye(3,3);zeros(3,3)];
    bar_A = [zeros(3,3),eye(3,3)];
elseif type == 'C'
    A = [];
    bar_A = [eye(6,6)];
else
    disp('undefined joint')
end