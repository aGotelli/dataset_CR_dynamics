function Ad_g = Ad(g)

R = g(1:3, 1:3);
r = g(1:3, 4);

Ad_g = [    R,     zeros(3, 3);
        hat(r)*R,       R    ];

end