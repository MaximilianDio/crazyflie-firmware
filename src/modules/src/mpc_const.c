#include "mpc_const.h" 

const real_t HoL[] = { /* Quadratic term (Hessian) matrix of QP over Lipschitz constant. */
0.4464593987846834, 0.0, 0.09377647594586541, 0.0, 0.08822008271323249, 0.0, 0.08266421049555717, 0.0, 0.07710911980031827, 0.0, 0.07155507113499458, 0.0, 0.06600232500706493, 0.0, 0.06045114192400813, 0.0, 0.05490178239330294, 0.0, 0.049354506922428196, 0.0, 
0.0, 0.4464593987846834, 0.0, 0.09377647594586541, 0.0, 0.08822008271323249, 0.0, 0.08266421049555717, 0.0, 0.07710911980031827, 0.0, 0.07155507113499458, 0.0, 0.06600232500706493, 0.0, 0.06045114192400813, 0.0, 0.05490178239330294, 0.0, 0.049354506922428196, 
0.09377647594586541, 0.0, 0.4402706936649991, 0.0, 0.0876279230859905, 0.0, 0.08211168211316693, 0.0, 0.07659596215530098, 0.0, 0.07108102371987143, 0.0, 0.06556712731435711, 0.0, 0.060054533446236825, 0.0, 0.05454350262298937, 0.0, 0.04903429535209355, 0.0, 
0.0, 0.09377647594586541, 0.0, 0.4402706936649991, 0.0, 0.0876279230859905, 0.0, 0.08211168211316693, 0.0, 0.07659596215530098, 0.0, 0.07108102371987143, 0.0, 0.06556712731435711, 0.0, 0.060054533446236825, 0.0, 0.05454350262298937, 0.0, 0.04903429535209355, 
0.08822008271323249, 0.0, 0.0876279230859905, 0.0, 0.4341620325574548, 0.0, 0.08155915373077671, 0.0, 0.0760828045102837, 0.0, 0.07060697630474828, 0.0, 0.06513192962164929, 0.0, 0.05965792496846553, 0.0, 0.054185222852675796, 0.0, 0.04871408378175889, 0.0, 
0.0, 0.08822008271323249, 0.0, 0.0876279230859905, 0.0, 0.4341620325574548, 0.0, 0.08155915373077671, 0.0, 0.0760828045102837, 0.0, 0.07060697630474828, 0.0, 0.06513192962164929, 0.0, 0.05965792496846553, 0.0, 0.054185222852675796, 0.0, 0.04871408378175889, 
0.08266421049555717, 0.0, 0.08211168211316694, 0.0, 0.08155915373077671, 0.0, 0.4281328944470927, 0.0, 0.07556964686526639, 0.0, 0.07013292888962513, 0.0, 0.06469673192894149, 0.0, 0.059261316490694244, 0.0, 0.05382694308236223, 0.0, 0.048393872211424244, 0.0, 
0.0, 0.08266421049555717, 0.0, 0.08211168211316694, 0.0, 0.08155915373077671, 0.0, 0.4281328944470927, 0.0, 0.07556964686526639, 0.0, 0.07013292888962513, 0.0, 0.06469673192894149, 0.0, 0.059261316490694244, 0.0, 0.05382694308236223, 0.0, 0.048393872211424244, 
0.07710911980031827, 0.0, 0.07659596215530098, 0.0, 0.0760828045102837, 0.0, 0.07556964686526639, 0.0, 0.42218275831895535, 0.0, 0.06965888147450199, 0.0, 0.06426153423623365, 0.0, 0.05886470801292295, 0.0, 0.05346866331204866, 0.0, 0.04807366064108959, 0.0, 
0.0, 0.07710911980031827, 0.0, 0.07659596215530098, 0.0, 0.0760828045102837, 0.0, 0.07556964686526639, 0.0, 0.42218275831895535, 0.0, 0.06965888147450199, 0.0, 0.06426153423623365, 0.0, 0.05886470801292295, 0.0, 0.05346866331204866, 0.0, 0.04807366064108959, 
0.07155507113499458, 0.0, 0.07108102371987143, 0.0, 0.07060697630474828, 0.0, 0.07013292888962512, 0.0, 0.06965888147450199, 0.0, 0.41631110315808506, 0.0, 0.06382633654352583, 0.0, 0.05846809953515166, 0.0, 0.05311038354173509, 0.0, 0.04775344907075493, 0.0, 
0.0, 0.07155507113499458, 0.0, 0.07108102371987143, 0.0, 0.07060697630474828, 0.0, 0.07013292888962512, 0.0, 0.06965888147450199, 0.0, 0.41631110315808506, 0.0, 0.06382633654352583, 0.0, 0.05846809953515166, 0.0, 0.05311038354173509, 0.0, 0.04775344907075493, 
0.06600232500706493, 0.0, 0.06556712731435713, 0.0, 0.06513192962164929, 0.0, 0.06469673192894149, 0.0, 0.06426153423623365, 0.0, 0.06382633654352585, 0.0, 0.4105174079495243, 0.0, 0.05807149105738037, 0.0, 0.052752103771421525, 0.0, 0.04743323750042029, 0.0, 
0.0, 0.06600232500706493, 0.0, 0.06556712731435713, 0.0, 0.06513192962164929, 0.0, 0.06469673192894149, 0.0, 0.06426153423623365, 0.0, 0.06382633654352585, 0.0, 0.4105174079495243, 0.0, 0.05807149105738037, 0.0, 0.052752103771421525, 0.0, 0.04743323750042029, 
0.06045114192400813, 0.0, 0.060054533446236825, 0.0, 0.059657924968465545, 0.0, 0.059261316490694244, 0.0, 0.05886470801292295, 0.0, 0.05846809953515166, 0.0, 0.05807149105738037, 0.0, 0.4048011516783153, 0.0, 0.05239382400110795, 0.0, 0.047113025930085636, 0.0, 
0.0, 0.06045114192400813, 0.0, 0.060054533446236825, 0.0, 0.059657924968465545, 0.0, 0.059261316490694244, 0.0, 0.05886470801292295, 0.0, 0.05846809953515166, 0.0, 0.05807149105738037, 0.0, 0.4048011516783153, 0.0, 0.05239382400110795, 0.0, 0.047113025930085636, 
0.05490178239330294, 0.0, 0.05454350262298937, 0.0, 0.054185222852675796, 0.0, 0.05382694308236223, 0.0, 0.05346866331204866, 0.0, 0.05311038354173509, 0.0, 0.052752103771421525, 0.0, 0.05239382400110795, 0.0, 0.3991618133295006, 0.0, 0.04679281435975099, 0.0, 
0.0, 0.05490178239330294, 0.0, 0.05454350262298937, 0.0, 0.054185222852675796, 0.0, 0.05382694308236223, 0.0, 0.05346866331204866, 0.0, 0.05311038354173509, 0.0, 0.052752103771421525, 0.0, 0.05239382400110795, 0.0, 0.3991618133295006, 0.0, 0.04679281435975099, 
0.049354506922428196, 0.0, 0.04903429535209355, 0.0, 0.04871408378175889, 0.0, 0.048393872211424244, 0.0, 0.04807366064108959, 0.0, 0.04775344907075493, 0.0, 0.04743323750042029, 0.0, 0.047113025930085636, 0.0, 0.04679281435975099, 0.0, 0.39359887188812254, 0.0, 
0.0, 0.049354506922428196, 0.0, 0.04903429535209355, 0.0, 0.04871408378175889, 0.0, 0.048393872211424244, 0.0, 0.04807366064108959, 0.0, 0.04775344907075493, 0.0, 0.04743323750042029, 0.0, 0.047113025930085636, 0.0, 0.04679281435975099, 0.0, 0.39359887188812254, 
 
};
const real_t GoL[] = { /* Linear term matrix of the QP, over Lipschitz constant. */
2.4537736690823717, 0.0, 14.553074191098668, 0.0, 
0.0, 2.4537736690823717, 0.0, 14.553074191098668, 
2.30720415933536, 0.0, 13.738795177244231, 0.0, 
0.0, 2.30720415933536, 0.0, 13.738795177244231, 
2.161585591170098, 0.0, 12.924573219884701, 0.0, 
0.0, 2.161585591170098, 0.0, 12.924573219884701, 
2.016917964586585, 0.0, 12.110446356683344, 0.0, 
0.0, 2.016917964586585, 0.0, 12.110446356683344, 
1.8732012795848219, 0.0, 11.296452625303434, 0.0, 
0.0, 1.8732012795848219, 0.0, 11.296452625303434, 
1.7304355361648076, 0.0, 10.482630063408237, 0.0, 
0.0, 1.7304355361648076, 0.0, 10.482630063408237, 
1.588620734326543, 0.0, 9.669016708661028, 0.0, 
0.0, 1.588620734326543, 0.0, 9.669016708661028, 
1.4477568740700275, 0.0, 8.85565059872507, 0.0, 
0.0, 1.4477568740700275, 0.0, 8.85565059872507, 
1.3078439553952617, 0.0, 8.04256977126364, 0.0, 
0.0, 1.3078439553952617, 0.0, 8.04256977126364, 
1.1688819783022448, 0.0, 7.229812263940003, 0.0, 
0.0, 1.1688819783022448, 0.0, 7.229812263940003, 

};
const real_t Bh_T[] = { /* Extended input matrix (used for reference tracking). */
0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 0.0, 0.001506707836661665, 0.0, 0.006848671984825749, 0.0, 0.0017806547160546947, 0.0, 0.006848671984825749, 0.0, 0.002054601595447725, 0.0, 0.006848671984825749, 0.0, 0.002328548474840755, 0.0, 0.006848671984825749, 0.0, 0.0026024953542337845, 0.0, 0.006848671984825749, 0.0, 
0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 0.0, 0.001506707836661665, 0.0, 0.006848671984825749, 0.0, 0.0017806547160546947, 0.0, 0.006848671984825749, 0.0, 0.002054601595447725, 0.0, 0.006848671984825749, 0.0, 0.002328548474840755, 0.0, 0.006848671984825749, 0.0, 0.0026024953542337845, 0.0, 0.006848671984825749, 
0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 0.0, 0.001506707836661665, 0.0, 0.006848671984825749, 0.0, 0.0017806547160546947, 0.0, 0.006848671984825749, 0.0, 0.002054601595447725, 0.0, 0.006848671984825749, 0.0, 0.002328548474840755, 0.0, 0.006848671984825749, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 0.0, 0.001506707836661665, 0.0, 0.006848671984825749, 0.0, 0.0017806547160546947, 0.0, 0.006848671984825749, 0.0, 0.002054601595447725, 0.0, 0.006848671984825749, 0.0, 0.002328548474840755, 0.0, 0.006848671984825749, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 0.0, 0.001506707836661665, 0.0, 0.006848671984825749, 0.0, 0.0017806547160546947, 0.0, 0.006848671984825749, 0.0, 0.002054601595447725, 0.0, 0.006848671984825749, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 0.0, 0.001506707836661665, 0.0, 0.006848671984825749, 0.0, 0.0017806547160546947, 0.0, 0.006848671984825749, 0.0, 0.002054601595447725, 0.0, 0.006848671984825749, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 0.0, 0.001506707836661665, 0.0, 0.006848671984825749, 0.0, 0.0017806547160546947, 0.0, 0.006848671984825749, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 0.0, 0.001506707836661665, 0.0, 0.006848671984825749, 0.0, 0.0017806547160546947, 0.0, 0.006848671984825749, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 0.0, 0.001506707836661665, 0.0, 0.006848671984825749, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 0.0, 0.001506707836661665, 0.0, 0.006848671984825749, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 0.0012327609572686349, 0.0, 0.006848671984825749, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 0.0009588140778756048, 0.0, 0.006848671984825749, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 0.0006848671984825749, 0.0, 0.006848671984825749, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 0.000410920319089545, 0.0, 0.006848671984825749, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000136973439696515, 0.0, 0.006848671984825749, 

};
const real_t E[] = { /* Linear factor (prediction matrix) of 2-sided state constraint. */
0, 

};
const real_t Kx_Ai[] = { /* Prediction component of the state constraint bound. */
0, 

};

const real_t u_lb[] = { /* Left (lower) constraint of the inputs for condensed QP. */
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 
-20, 

};
const real_t u_ub[] = { /* Right (upper) constraint of the inputs for condensed QP. */
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 
20, 

};
const real_t e_lb[] = { /* Left (lower) constraint of the states for condensed QP. */
0, 

};
const real_t e_ub[] = { /* Right (upper) constraint of the states for condensed QP. */
0, 

};

const real_t nu = 0.25767541480865974; /* Fast gradient extra step constant */
const real_t mu = 0.0; /* Augmented Lagrange multiplier penalty parameter. */
const real_t Linv = 3.471262690987062; /* Inverse of gradient Lipschitz constant (1/L) */

/* state dependent variables */
real_t gxoL[MPC_HOR_INPUTS];  /* gradient vector as a function of the current state */
real_t zx_lb[MPC_HOR_MXCONSTRS];  /* mixed constraint lower bound as function of current state */
real_t zx_ub[MPC_HOR_MXCONSTRS];  /* mixed constraint upper bound as function of current state */

/* reference dependent variables */
real_t groL[MPC_HOR_INPUTS];

/* MPC system: state-space and weighting matrices */
const real_t Q[] = {  /* State weighting matrix */
1, 0, 0, 0, 
0, 1, 0, 0, 
0, 0, 30, 0, 
0, 0, 0, 30, 

};
const real_t R[] = {   /* Input weighting matrix */
0.1, 0.0, 
0.0, 0.1, 

};
const real_t P[] = {   /* Terminal state weighting matrix */
145.6309293441364, 0.0, 46.25473304588374, 0.0, 
0.0, 145.6309293441364, 0.0, 46.25473304588374, 
46.25473304588374, 0.0, 283.51969574057955, 0.0, 
0.0, 46.25473304588374, 0.0, 283.51969574057955, 

};
const real_t K[] = {   /* Linear quadratic regulator gain matrix */
-2.9697289849816704, 0.0, -17.299375679325127, 0.0, 
0.0, -2.9697289849816704, 0.0, -17.299375679325127, 

};
const real_t Ad[] = {   /* Discrete-time system matrix */
1.0, 0.0, 0.04, 0.0, 
0.0, 1.0, 0.0, 0.04, 
0.0, 0.0, 1.0, 0.0, 
0.0, 0.0, 0.0, 1.0, 

};
const real_t Bd[] = {   /* Discrete-time input matrix */
0.000136973439696515, 0.0, 
0.0, 0.000136973439696515, 
0.006848671984825749, 0.0, 
0.0, 0.006848671984825749, 

};
const real_t dt = 0.04;


/* User variables created with appropriate size */
real_t u_opt[MPC_HOR_INPUTS];  /* Optimal input */
real_t l_opt[MPC_HOR_MXCONSTRS];  /* Optimal multiplier */
real_t x_trj[MPC_HOR_STATES + MPC_STATES];  /* State trajectory for u_opt */
real_t u_ini[MPC_HOR_INPUTS];  /* Initial guess input */
real_t l_ini[MPC_HOR_MXCONSTRS];  /* Initial guess multiplier */
real_t ctl_x_ref[MPC_HOR_STATES];  /* Reference for state trajectory */
real_t ctl_u_ref[MPC_HOR_INPUTS];  /* Reference for input trajectory */

/* Always check this declarations match the structure definitions */
struct mpc_conf conf = {1, 1, 0};

struct mpc_qpx qpx = {HoL, gxoL, E, u_lb, u_ub, zx_lb, zx_ub,
		MPC_HOR_INPUTS, MPC_HOR_MXCONSTRS};

struct mpc_sys sys = {Ad, Bd, &dt};

struct mpc_wmx wmx = {Q, R, P,};

struct mpc_lqr lqr = {K, P,};

struct mpc_fgm fgm = {u_ini, gxoL, groL, &(conf.in_iter),
			HoL, GoL, Bh_T, u_lb, u_ub, &nu,
			MPC_HOR, MPC_STATES, MPC_INPUTS, MPC_HOR_INPUTS, MPC_HOR_STATES};

struct mpc_alm alm = {&fgm, l_ini, zx_lb, zx_ub, &(conf.ex_iter),
			&mu,
			E, Kx_Ai, e_lb, e_ub,
			&Linv, 
			MPC_STATES, MPC_MXCONSTRS, MPC_HOR_INPUTS, MPC_HOR_MXCONSTRS};

struct mpc_ctl ctl = {&conf, &qpx, &sys, &wmx, &lqr, &alm,
			u_opt, l_opt, x_trj, 0, 0, u_ini, l_ini};

