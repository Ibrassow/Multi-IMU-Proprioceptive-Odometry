function G_mtx = G(q)
   H = [zeros(1, 3); I];
   G_mtx = L(q) * H;
end