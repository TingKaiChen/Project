clc
[lt_x, lt_y, lt_utm] = deg2utm(24.7756600,121.0457539);
[lb_x, lb_y, lb_utm] = deg2utm(24.7749817,121.0452239);
[rb_x, rb_y, rb_utm] = deg2utm(24.7741421,121.0463000);
[rt_x, rt_y, rt_utm] = deg2utm(24.7748169,121.0468272);

left = norm([lt_x-lb_x, lt_y-lb_y])
bottom = norm([lb_x-rb_x, lb_y-rb_y])
right = norm([rb_x-rt_x, rb_y-rt_y])
top = norm([rt_x-lt_x, rt_y-lt_y])