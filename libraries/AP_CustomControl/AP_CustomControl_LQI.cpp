#include "AP_CustomControl_LQI.h"

#if AP_PLANE_CUSTOMCONTROL_LQI_ENABLED
#include "AP_CustomControl_LQI_gains.h"
using namespace AP_CustomControl_MIMO;
using namespace MicroAGU_LQI;

// AI-assisted research table: constrained nonlinear steady turns, as-built model.
struct TurnTrim { float speed; float x[16]; float u[3]; };
static constexpr float trim_rates[] = {-4.000000000e+01f,-3.000000000e+01f,-2.500000000e+01f,-2.000000000e+01f,-1.500000000e+01f,-1.000000000e+01f,0.000000000e+00f,1.000000000e+01f,1.500000000e+01f,2.000000000e+01f,2.500000000e+01f,3.000000000e+01f,4.000000000e+01f};
static constexpr uint8_t trim_offsets[] = {0,5,10,15,19,23,28,32,36,39,42,46,50,55};
static constexpr TurnTrim trim_points[] = {{3.800f,{-2.303908314e+00f,5.900027711e-01f,-1.894541340e-01f,4.835525076e-01f,-4.624440201e-01f,-1.047100200e+00f,-1.479220964e+00f,-5.169838633e-03f,3.180709161e-01f,5.549276762e-01f,-1.169899117e+00f,4.343747444e-01f,6.202404881e-01f,-1.097450726e+00f,1.288883411e+00f,0.000000000e+00f},{5.330943299e-01f,6.109238966e-01f,-1.951899249e-01f}},{4.000f,{-2.326560988e+00f,5.008625984e-01f,-1.872588415e-01f,3.081796515e-01f,-4.177476599e-01f,-6.182939571e-01f,-1.421375588e+00f,-6.390047459e-03f,2.978882622e-01f,5.616193191e-01f,-1.172017108e+00f,3.847358969e-01f,6.209379634e-01f,-1.115433599e+00f,9.322555966e-01f,0.000000000e+00f},{5.106920697e-01f,4.148691713e-01f,-2.282538450e-01f}},{4.200f,{-2.349663001e+00f,4.284144326e-01f,-1.817681587e-01f,1.668405726e-01f,-3.791063122e-01f,-1.951015800e-01f,-1.367809016e+00f,2.387913153e-03f,2.814315867e-01f,5.679364734e-01f,-1.173039727e+00f,3.442671049e-01f,6.218621455e-01f,-1.128071545e+00f,6.582859319e-01f,0.000000000e+00f},{4.935427547e-01f,2.606109066e-01f,-2.544765160e-01f}},{4.300f,{-2.359722106e+00f,3.963208910e-01f,-1.776880376e-01f,1.055542445e-01f,-3.612124681e-01f,1.530520173e-02f,-1.339624369e+00f,1.021897001e-02f,2.741269679e-01f,5.706755332e-01f,-1.173440079e+00f,3.265783559e-01f,6.219581295e-01f,-1.133266271e+00f,5.440418529e-01f,0.000000000e+00f},{4.864075262e-01f,1.923115197e-01f,-2.672780716e-01f}},{4.400f,{-2.367272515e+00f,3.660066365e-01f,-1.723369484e-01f,4.917866485e-02f,-3.436240736e-01f,2.254260334e-01f,-1.307754771e+00f,2.083250712e-02f,2.672194193e-01f,5.728494117e-01f,-1.173974068e+00f,3.102317289e-01f,6.213777434e-01f,-1.138166838e+00f,4.431585753e-01f,0.000000000e+00f},{4.801147011e-01f,1.273631735e-01f,-2.811345776e-01f}},{3.800f,{-1.715606416e+00f,3.795687926e-01f,-1.369844611e-01f,3.361026554e-01f,-2.850901926e-01f,-9.488991434e-01f,-9.859746060e-01f,2.329482835e-02f,2.027330192e-01f,3.267742950e-01f,-9.231023145e-01f,2.933640316e-01f,3.576872473e-01f,-8.865649316e-01f,9.022673764e-01f,0.000000000e+00f},{5.088123367e-01f,4.620100314e-01f,-1.809386946e-01f}},{4.000f,{-1.731475767e+00f,3.094687301e-01f,-1.344529496e-01f,1.889569598e-01f,-2.574832534e-01f,-5.319214581e-01f,-9.470497827e-01f,2.083090169e-02f,1.907374424e-01f,3.301392977e-01f,-9.244605304e-01f,2.601112181e-01f,3.583670087e-01f,-8.966132060e-01f,5.970880366e-01f,0.000000000e+00f},{4.897193973e-01f,3.013582913e-01f,-1.987936249e-01f}},{4.200f,{-1.744149229e+00f,2.512227343e-01f,-1.286138962e-01f,6.821704485e-02f,-2.318111719e-01f,-1.175878030e-01f,-9.041551558e-01f,2.789347886e-02f,1.807485483e-01f,3.328223126e-01f,-9.255048733e-01f,2.326557395e-01f,3.582721581e-01f,-9.041639052e-01f,3.569712709e-01f,0.000000000e+00f},{4.747445333e-01f,1.653306295e-01f,-2.167883684e-01f}},{4.300f,{-1.746612802e+00f,2.250827186e-01f,-1.237881886e-01f,1.546545222e-02f,-2.188130267e-01f,8.922336675e-02f,-8.768021555e-01f,3.554632790e-02f,1.762595334e-01f,3.334961409e-01f,-9.261279074e-01f,2.206260946e-01f,3.573487113e-01f,-9.075384426e-01f,2.579720272e-01f,0.000000000e+00f},{4.685829269e-01f,1.019140069e-01f,-2.278694406e-01f}},{4.400f,{-1.743766046e+00f,2.001212953e-01f,-1.168855314e-01f,-3.325510606e-02f,-2.048975242e-01f,2.962161951e-01f,-8.411173494e-01f,4.686649447e-02f,1.719694920e-01f,3.332925921e-01f,-9.270073041e-01f,2.095098088e-01f,3.552391707e-01f,-9.109943861e-01f,1.741173872e-01f,0.000000000e+00f},{4.633696883e-01f,3.933133320e-02f,-2.419638291e-01f}},{3.800f,{-1.421902008e+00f,3.009631691e-01f,-1.104517468e-01f,2.885455336e-01f,-2.143036772e-01f,-9.158384701e-01f,-7.713009173e-01f,2.952130344e-02f,1.577333218e-01f,2.331773721e-01f,-7.843423722e-01f,2.341952370e-01f,2.524339407e-01f,-7.588637169e-01f,7.636975837e-01f,0.000000000e+00f},{5.001347234e-01f,4.174610911e-01f,-1.601106960e-01f}},{4.000f,{-1.434602397e+00f,2.373416771e-01f,-1.086643007e-01f,1.508692511e-01f,-1.938622083e-01f,-5.027361627e-01f,-7.418453275e-01f,2.576867665e-02f,1.486376851e-01f,2.354037352e-01f,-7.854529677e-01f,2.076744865e-01f,2.530586331e-01f,-7.663400552e-01f,4.741760947e-01f,0.000000000e+00f},{4.820486917e-01f,2.669725194e-01f,-1.726799788e-01f}},{4.200f,{-1.442919087e+00f,1.845482029e-01f,-1.034297481e-01f,3.768057827e-02f,-1.738233604e-01f,-9.157299912e-02f,-7.056770273e-01f,3.158260450e-02f,1.410761125e-01f,2.369286465e-01f,-7.863884482e-01f,1.857906715e-01f,2.527396549e-01f,-7.720420569e-01f,2.465878607e-01f,0.000000000e+00f},{4.678748614e-01f,1.364548992e-01f,-1.872538534e-01f}},{4.300f,{-1.442405530e+00f,1.610116287e-01f,-9.870904676e-02f,-1.169065018e-02f,-1.630990325e-01f,1.137935333e-01f,-6.802988776e-01f,3.877912946e-02f,1.377010868e-01f,2.370232920e-01f,-7.869579494e-01f,1.762449372e-01f,2.516676996e-01f,-7.746260617e-01f,1.543073964e-01f,0.000000000e+00f},{4.621388826e-01f,7.458382602e-02f,-1.972839428e-01f}},{4.400f,{-1.435547812e+00f,1.386861317e-01f,-9.161989268e-02f,-5.716814130e-02f,-1.510282081e-01f,3.193756182e-01f,-6.450608504e-01f,4.982571874e-02f,1.344975751e-01f,2.362335457e-01f,-7.877490455e-01f,1.674700998e-01f,2.493850785e-01f,-7.773064342e-01f,7.803933895e-02f,0.000000000e+00f},{4.574031089e-01f,1.277113422e-02f,-2.109590989e-01f}},{3.800f,{-1.129613875e+00f,2.368885764e-01f,-8.422450953e-02f,2.545752755e-01f,-1.532095971e-01f,-8.905602600e-01f,-5.751977520e-01f,3.135836194e-02f,1.188582725e-01f,1.546261873e-01f,-6.375012923e-01f,1.806909107e-01f,1.652412142e-01f,-6.200730444e-01f,6.547789663e-01f,0.000000000e+00f},{4.933235825e-01f,3.877621343e-01f,-1.331258773e-01f}},{4.000f,{-1.139747216e+00f,1.783169077e-01f,-8.367909361e-02f,1.242277076e-01f,-1.394206221e-01f,-4.804082313e-01f,-5.559935563e-01f,2.616413178e-02f,1.121462445e-01f,1.560337820e-01f,-6.383738115e-01f,1.602447410e-01f,1.658549345e-01f,-6.255063614e-01f,3.769775353e-01f,0.000000000e+00f},{4.759905932e-01f,2.447910799e-01f,-1.408580654e-01f}},{4.200f,{-1.145209074e+00f,1.301496939e-01f,-7.975664207e-02f,1.721732720e-02f,-1.249107681e-01f,-7.184887685e-02f,-5.288963654e-01f,3.044499772e-02f,1.066177035e-01f,1.568614411e-01f,-6.391178280e-01f,1.434368444e-01f,1.656141938e-01f,-6.296366054e-01f,1.598008981e-01f,0.000000000e+00f},{4.624801669e-01f,1.188049903e-01f,-1.514746172e-01f}},{4.300f,{-1.143146184e+00f,1.090290214e-01f,-7.568738387e-02f,-2.923665260e-02f,-1.166354369e-01f,1.322210401e-01f,-5.077601333e-01f,3.684864716e-02f,1.041911135e-01f,1.567238000e-01f,-6.395516452e-01f,1.361616145e-01f,1.647205706e-01f,-6.314836441e-01f,7.384591261e-02f,0.000000000e+00f},{4.571428527e-01f,5.851575280e-02f,-1.597076944e-01f}},{3.800f,{-8.392107787e-01f,1.854639058e-01f,-5.839730444e-02f,2.320865217e-01f,-1.003366587e-01f,-8.714445672e-01f,-3.953051197e-01f,2.971311446e-02f,8.472442314e-02f,9.174258615e-02f,-4.841539735e-01f,1.316025612e-01f,9.636057616e-02f,-4.726472313e-01f,5.722837232e-01f,0.000000000e+00f},{4.881705158e-01f,3.706446198e-01f,-1.015935527e-01f}},{4.000f,{-8.475789284e-01f,1.306859057e-01f,-5.958088606e-02f,1.072696019e-01f,-9.292925329e-02f,-4.634907490e-01f,-3.875246137e-01f,2.304242424e-02f,8.000946829e-02f,9.259734447e-02f,-4.847929628e-01f,1.167201593e-01f,9.698793718e-02f,-4.764134176e-01f,3.032812714e-01f,0.000000000e+00f},{4.714020121e-01f,2.334816375e-01f,-1.044764061e-01f}},{4.200f,{-8.521767311e-01f,8.627937443e-02f,-5.777190944e-02f,5.184437270e-03f,-8.408055648e-02f,-5.703423909e-02f,-3.725053230e-01f,2.558075240e-02f,7.618183871e-02f,9.310037964e-02f,-4.853129369e-01f,1.045551214e-01f,9.704023095e-02f,-4.792195949e-01f,9.449888909e-02f,0.000000000e+00f},{4.584247673e-01f,1.116317063e-01f,-1.101618478e-01f}},{4.300f,{-8.504578523e-01f,6.727246006e-02f,-5.501754804e-02f,-3.882042055e-02f,-7.856863035e-02f,1.459011340e-01f,-3.584336866e-01f,3.090801906e-02f,7.454212212e-02f,9.300180774e-02f,-4.855863834e-01f,9.934035193e-02f,9.655929937e-02f,-4.804245999e-01f,1.377591327e-02f,0.000000000e+00f},{4.534162234e-01f,5.321968199e-02f,-1.155567812e-01f}},{3.800f,{-5.508030341e-01f,1.451609658e-01f,-3.291549102e-02f,2.194294788e-01f,-5.435690561e-02f,-8.572355024e-01f,-2.290307118e-01f,2.547801622e-02f,5.417074761e-02f,4.489711592e-02f,-3.258234034e-01f,8.579764191e-02f,4.587762187e-02f,-3.188183175e-01f,5.140693172e-01f,0.000000000e+00f},{4.845371391e-01f,3.644739913e-01f,-6.691430161e-02f}},{4.000f,{-5.582473056e-01f,9.296332140e-02f,-3.627143270e-02f,9.853316943e-02f,-5.322808345e-02f,-4.508223029e-01f,-2.339881061e-01f,1.740101991e-02f,5.117217401e-02f,4.538653077e-02f,-3.262400689e-01f,7.609371616e-02f,4.646890015e-02f,-3.211876380e-01f,2.513303155e-01f,0.000000000e+00f},{4.681698179e-01f,2.319529945e-01f,-6.470674984e-02f}},{4.200f,{-5.640593119e-01f,5.129468356e-02f,-3.735925553e-02f,1.405875502e-04f,-5.028864700e-02f,-4.596805276e-02f,-2.341378042e-01f,1.815100491e-02f,4.877539487e-02f,4.576788526e-02f,-3.265537822e-01f,6.820966214e-02f,4.681947784e-02f,-3.229032202e-01f,4.893496226e-02f,0.000000000e+00f},{4.555970860e-01f,1.141078793e-01f,-6.435218355e-02f}},{4.300f,{-5.646742966e-01f,3.391946143e-02f,-3.658732901e-02f,-4.192569736e-02f,-4.790097378e-02f,1.560357301e-01f,-2.299946271e-01f,2.226677906e-02f,4.777520306e-02f,4.582327083e-02f,-3.266938459e-01f,6.486349898e-02f,4.678076993e-02f,-3.235975852e-01f,-2.771462349e-02f,0.000000000e+00f},{4.508438616e-01f,5.792967438e-02f,-6.579909294e-02f}},{4.400f,{-5.617945821e-01f,1.865636375e-02f,-3.435154400e-02f,-7.980914793e-02f,-4.447621568e-02f,3.578693365e-01f,-2.201915510e-01f,2.943503670e-02f,4.689623133e-02f,4.567651987e-02f,-3.268417203e-01f,6.188040325e-02f,4.644964056e-02f,-3.242288662e-01f,-8.900851047e-02f,0.000000000e+00f},{4.470461725e-01f,2.114776469e-03f,-6.935877523e-02f}},{3.800f,{2.108939500e-02f,9.350660874e-02f,1.782153645e-02f,2.189748937e-01f,2.170471190e-02f,-8.400351141e-01f,7.392623130e-02f,1.227962026e-02f,-1.490575982e-14f,-4.993634309e-14f,-9.839110468e-16f,-1.542027083e-15f,2.685241434e-14f,1.588045546e-17f,4.649068887e-01f,0.000000000e+00f},{4.814706679e-01f,3.736307922e-01f,6.645821116e-03f}},{4.000f,{1.383479894e-02f,4.271284837e-02f,9.211592772e-03f,1.014086540e-01f,1.023660244e-02f,-4.348854071e-01f,4.035111555e-02f,1.991878246e-03f,-1.597103544e-14f,-4.903585743e-14f,-9.582891524e-16f,4.130078585e-16f,1.038219227e-14f,2.371676067e-16f,2.077106412e-01f,0.000000000e+00f},{4.654575388e-01f,2.347139401e-01f,1.970613362e-02f}},{4.200f,{1.312799738e-03f,2.719637276e-03f,6.974814217e-04f,6.590968920e-03f,7.203407704e-04f,-3.162320513e-02f,3.224710812e-03f,-1.770150246e-04f,-1.545329195e-14f,-5.043228146e-14f,-1.027159453e-15f,-8.167897578e-16f,9.916311983e-15f,1.941619721e-16f,1.242000600e-02f,0.000000000e+00f},{4.533321377e-01f,1.095089617e-01f,3.453851469e-02f}},{4.300f,{-8.175321049e-03f,-1.354210555e-02f,-3.902258452e-03f,-3.330694855e-02f,-3.909360143e-03f,1.692961673e-01f,-1.849242300e-02f,1.848793513e-03f,-1.486853507e-14f,-6.064661717e-14f,-1.470592405e-15f,5.035860858e-15f,-7.882901314e-15f,5.745949981e-17f,-5.986679884e-02f,0.000000000e+00f},{4.488513886e-01f,4.945527061e-02f,4.339231597e-02f}},{3.800f,{5.898450820e-01f,7.651185407e-02f,7.030626105e-02f,2.474608482e-01f,8.379636364e-02f,-8.345007524e-01f,3.569810187e-01f,-3.671244241e-03f,-5.022620227e-02f,2.046601509e-02f,3.288914440e-01f,-8.416449920e-02f,2.722256126e-02f,3.213819851e-01f,5.009265556e-01f,0.000000000e+00f},{4.837171946e-01f,3.508473104e-01f,7.729094463e-02f}},{4.000f,{5.834219399e-01f,2.192292246e-02f,5.627355505e-02f,1.273528704e-01f,5.944000303e-02f,-4.282614656e-01f,2.936211211e-01f,-1.501834177e-02f,-4.708437422e-02f,2.012557093e-02f,3.293768878e-01f,-7.442920995e-02f,2.573168126e-02f,3.238970275e-01f,2.402057299e-01f,0.000000000e+00f},{4.674779524e-01f,2.045544528e-01f,1.008713840e-01f}},{4.200f,{5.648882879e-01f,-2.204369636e-02f,4.020602987e-02f,3.077166451e-02f,3.744557159e-02f,-2.430260312e-02f,2.187436389e-01f,-1.839152487e-02f,-4.455078510e-02f,1.908032884e-02f,3.297912669e-01f,-6.655330626e-02f,2.349537315e-02f,3.257755914e-01f,4.272243024e-02f,0.000000000e+00f},{4.552116625e-01f,7.048621196e-02f,1.298722121e-01f}},{4.300f,{5.467623241e-01f,-4.058760292e-02f,3.013861702e-02f,-9.543245916e-03f,2.587494653e-02f,1.767579878e-01f,1.708403760e-01f,-1.718708810e-02f,-4.348139194e-02f,1.804469323e-02f,3.299922252e-01f,-6.324862170e-02f,2.175460136e-02f,3.265539164e-01f,-2.953458570e-02f,0.000000000e+00f},{4.507310569e-01f,4.204027646e-03f,1.490310073e-01f}},{3.800f,{8.742773994e-01f,8.079077740e-02f,9.801555135e-02f,2.721926426e-01f,1.121256754e-01f,-8.356947325e-01f,4.981923488e-01f,-1.193217516e-02f,-7.570842561e-02f,5.513549339e-02f,4.911499890e-01f,-1.278817758e-01f,6.814299112e-02f,4.785423536e-01f,5.509467842e-01f,0.000000000e+00f},{4.868385104e-01f,3.543416179e-01f,1.084491713e-01f}},{4.000f,{8.695917307e-01f,2.236865809e-02f,8.159693134e-02f,1.486018316e-01f,8.110095742e-02f,-4.280214445e-01f,4.205862744e-01f,-2.342252487e-02f,-7.066506888e-02f,5.481645515e-02f,4.919365856e-01f,-1.129233092e-01f,6.573806007e-02f,4.826249410e-01f,2.849034071e-01f,0.000000000e+00f},{4.702583778e-01f,2.033728857e-01f,1.366731286e-01f}},{4.200f,{8.508403791e-01f,-2.555024736e-02f,6.250626430e-02f,4.907317485e-02f,5.314592600e-02f,-2.285970331e-02f,3.296013606e-01f,-2.678069799e-02f,-6.652290624e-02f,5.326785577e-02f,4.926837104e-01f,-1.007526200e-01f,6.202887588e-02f,4.857996790e-01f,8.255438259e-02f,0.000000000e+00f},{4.576833408e-01f,6.502158863e-02f,1.711922846e-01f}},{3.800f,{1.159015960e+00f,9.409625190e-02f,1.270479890e-01f,3.043196882e-01f,1.399416816e-01f,-8.396971778e-01f,6.426737888e-01f,-2.023220578e-02f,-1.024746059e-01f,1.059162791e-01f,6.501731626e-01f,-1.739644032e-01f,1.271616997e-01f,6.308809183e-01f,6.233271774e-01f,0.000000000e+00f},{4.913583628e-01f,3.690227495e-01f,1.352999005e-01f}},{4.000f,{1.157241349e+00f,3.056731122e-02f,1.085873766e-01f,1.757326983e-01f,1.019437161e-01f,-4.299652286e-01f,5.514330441e-01f,-3.171567286e-02f,-9.516437635e-02f,1.058907362e-01f,6.512874464e-01f,-1.533745160e-01f,1.239047598e-01f,6.368424553e-01f,3.493494051e-01f,0.000000000e+00f},{4.742698955e-01f,2.125482264e-01f,1.677057871e-01f}},{4.200f,{1.140903453e+00f,-2.238010791e-02f,8.724573954e-02f,7.190319057e-02f,6.821193333e-02f,-2.306286910e-02f,4.470696929e-01f,-3.483287644e-02f,-8.906277798e-02f,1.041673709e-01f,6.524273330e-01f,-1.365080394e-01f,1.188991207e-01f,6.416174863e-01f,1.397781675e-01f,0.000000000e+00f},{4.612363640e-01f,7.023980263e-02f,2.062477941e-01f}},{3.800f,{1.443859601e+00f,1.171652880e-01f,1.575636730e-01f,3.444890375e-01f,1.682397496e-01f,-8.468616041e-01f,7.927549392e-01f,-2.852810270e-02f,-1.314075906e-01f,1.725094833e-01f,8.046222516e-01f,-2.234685064e-01f,2.040651521e-01f,7.764429695e-01f,7.193181100e-01f,0.000000000e+00f},{4.973584573e-01f,3.961217886e-01f,1.567307751e-01f}},{4.000f,{1.446116457e+00f,4.720427133e-02f,1.374140808e-01f,2.093134694e-01f,1.227499115e-01f,-4.343794417e-01f,6.882331690e-01f,-3.996372637e-02f,-1.213499334e-01f,1.731505019e-01f,8.060629887e-01f,-1.967187542e-01f,2.001272765e-01f,7.846625067e-01f,4.346394917e-01f,0.000000000e+00f},{4.795836475e-01f,2.328787738e-01f,1.930685787e-01f}},{4.200f,{1.434755833e+00f,-1.167460471e-02f,1.146134800e-01f,9.988314061e-02f,8.326958643e-02f,-2.522264514e-02f,5.730726365e-01f,-4.278927206e-02f,-1.128713723e-01f,1.718023180e-01f,8.075818604e-01f,-1.746725795e-01f,1.942868037e-01f,7.913195134e-01f,2.153749985e-01f,0.000000000e+00f},{4.659340055e-01f,8.676829772e-02f,2.342173040e-01f}},{4.300f,{1.418529532e+00f,-3.840913433e-02f,1.005754498e-01f,5.352372404e-02f,6.372155328e-02f,1.783321672e-01f,5.032673775e-01f,-4.097560917e-02f,-1.090176102e-01f,1.696208244e-01f,8.085718156e-01f,-1.651353083e-01f,1.895743986e-01f,7.945038212e-01f,1.308219238e-01f,0.000000000e+00f},{4.606801054e-01f,1.491508732e-02f,2.602620900e-01f}},{3.800f,{1.728270157e+00f,1.510102739e-01f,1.896302201e-01f,3.936241018e-01f,1.980889412e-01f,-8.577317496e-01f,9.505956588e-01f,-3.675217275e-02f,-1.635194836e-01f,2.544208264e-01f,9.531691464e-01f,-2.775535208e-01f,2.984578271e-01f,9.131741174e-01f,8.407829577e-01f,0.000000000e+00f},{5.049603334e-01f,4.372818632e-01f,1.715443098e-01f}},{4.000f,{1.735421474e+00f,7.311663605e-02f,1.680902309e-01f,2.500963463e-01f,1.442813911e-01f,-4.416753012e-01f,8.325491253e-01f,-4.822000463e-02f,-1.500927686e-01f,2.561626646e-01f,9.549098649e-01f,-2.439866186e-01f,2.940491846e-01f,9.241242379e-01f,5.422279974e-01f,0.000000000e+00f},{4.862943168e-01f,2.652231376e-01f,2.120292767e-01f}},{4.200f,{1.731031499e+00f,7.468347969e-03f,1.444973565e-01f,1.338049340e-01f,9.880401467e-02f,-2.973931157e-02f,7.083608346e-01f,-5.084608283e-02f,-1.387551715e-01f,2.558584717e-01f,9.567044710e-01f,-2.162211985e-01f,2.879784194e-01f,9.329077250e-01f,3.109971556e-01f,0.000000000e+00f},{4.718822388e-01f,1.149426570e-01f,2.548192271e-01f}},{4.300f,{1.719760130e+00f,-2.254694833e-02f,1.304587688e-01f,8.418005483e-02f,7.707354952e-02f,1.751014135e-01f,6.363372887e-01f,-4.887458228e-02f,-1.335653088e-01f,2.541588550e-01f,9.578953934e-01f,-2.040989602e-01f,2.831768271e-01f,9.370989804e-01f,2.191922564e-01f,0.000000000e+00f},{4.661713279e-01f,4.240252800e-02f,2.805970405e-01f}},{3.800f,{2.291821920e+00f,2.566372964e-01f,2.583312305e-01f,5.241267425e-01f,2.674985969e-01f,-8.938939690e-01f,1.297676653e+00f,-5.193165332e-02f,-2.422372290e-01f,4.608981675e-01f,1.227465593e+00f,-4.047235782e-01f,5.369133897e-01f,1.151390731e+00f,1.173223433e+00f,0.000000000e+00f},{5.258193144e-01f,5.723660566e-01f,1.750446602e-01f}},{4.000f,{2.310199958e+00f,1.566841996e-01f,2.344409309e-01f,3.570260407e-01f,1.927980340e-01f,-4.671634114e-01f,1.148422613e+00f,-6.448185222e-02f,-2.193284812e-01f,4.662603386e-01f,1.229745539e+00f,-3.548056268e-01f,5.319763605e-01f,1.169996537e+00f,8.317746525e-01f,0.000000000e+00f},{5.043961874e-01f,3.708692139e-01f,2.276584909e-01f}},{4.200f,{2.322247237e+00f,7.427376565e-02f,2.103601201e-01f,2.229080761e-01f,1.330470850e-01f,-4.765386417e-02f,1.006162077e+00f,-6.727051995e-02f,-2.003899133e-01f,4.699211014e-01f,1.231582648e+00f,-3.137291711e-01f,5.269531323e-01f,1.183922371e+00f,5.698809594e-01f,0.000000000e+00f},{4.880205014e-01f,2.072934667e-01f,2.740257838e-01f}},{4.300f,{2.323091658e+00f,3.708680346e-02f,1.972212603e-01f,1.653434066e-01f,1.063692549e-01f,1.603437860e-01f,9.317863286e-01f,-6.549187968e-02f,-1.918302471e-01f,4.705565315e-01f,1.232702513e+00f,-2.956833988e-01f,5.234014412e-01f,1.190126059e+00f,4.623751150e-01f,0.000000000e+00f},{4.813127985e-01f,1.326074671e-01f,2.982546734e-01f}},{4.400f,{2.317698598e+00f,1.351871028e-03f,1.824727878e-01f,1.131885910e-01f,8.059193174e-02f,3.674917476e-01f,8.505234950e-01f,-6.146317458e-02f,-1.835971900e-01f,4.698125170e-01f,1.234239056e+00f,-2.789777280e-01f,5.183004437e-01f,1.196375299e+00f,3.702145463e-01f,0.000000000e+00f},{4.755693373e-01f,5.962982837e-02f,3.252922913e-01f}}};
static float turn_trim(float rate, float speed, float *x, float *u)
{
    constexpr uint8_t count = sizeof(trim_rates)/sizeof(trim_rates[0]);
    rate = constrain_float(rate, trim_rates[0], trim_rates[count-1]);
    uint8_t lo=0;
    while (lo+2 < count && rate > trim_rates[lo+1]) { lo++; }
    const float mix=(rate-trim_rates[lo])/(trim_rates[lo+1]-trim_rates[lo]);
    const float maximum=(1-mix)*trim_points[trim_offsets[lo+1]-1].speed+mix*trim_points[trim_offsets[lo+2]-1].speed;
    speed=constrain_float(speed, 3.8f, maximum);
    const float speed_fraction=(speed-3.8f)/(maximum-3.8f);
    for (uint8_t j=0; j<16; j++) { x[j]=0; }
    for (uint8_t j=0; j<3; j++) { u[j]=0; }
    for (uint8_t side=0; side<2; side++) {
        uint8_t a=trim_offsets[lo+side];
        const uint8_t end=trim_offsets[lo+side+1];
        const float row_speed=3.8f+speed_fraction*(trim_points[end-1].speed-3.8f);
        while (a+2<end && row_speed>trim_points[a+1].speed) { a++; }
        const float f=constrain_float((row_speed-trim_points[a].speed)/(trim_points[a+1].speed-trim_points[a].speed),0,1);
        const float w=side ? mix : 1-mix;
        for (uint8_t j=0; j<16; j++) { x[j]+=w*((1-f)*trim_points[a].x[j]+f*trim_points[a+1].x[j]); }
        for (uint8_t j=0; j<3; j++) { u[j]+=w*((1-f)*trim_points[a].u[j]+f*trim_points[a+1].u[j]); }
    }
    return speed;
}



// AI-assisted: configurable reference limits; defaults preserve the prior governor.
const AP_Param::GroupInfo AP_CustomControl_LQI::var_info[] = {
    // @Param: HUP
    // @DisplayName: LQI height reference climb limit
    // @Description: Maximum climb rate of the LQI internal altitude reference.
    // @Units: m/s
    // @Range: 0.1 3
    // @User: Advanced
    AP_GROUPINFO("HUP", 1, AP_CustomControl_LQI, _height_up, 0.5f),
    // @Param: HDN
    // @DisplayName: LQI height reference descent limit
    // @Description: Positive maximum descent rate of the LQI internal altitude reference.
    // @Units: m/s
    // @Range: 0.1 3
    // @User: Advanced
    AP_GROUPINFO("HDN", 2, AP_CustomControl_LQI, _height_down, 0.5f),
    // @Param: HTC
    // @DisplayName: LQI height reference time constant
    // @Description: Time constant for altitude reference approach near the requested height.
    // @Units: s
    // @Range: 0.2 10
    // @User: Advanced
    AP_GROUPINFO("HTC", 3, AP_CustomControl_LQI, _height_tc, 2.0f),
    // @Param: HRTC
    // @DisplayName: LQI climb reference smoothing
    // @Description: Time constant for climb reference smoothing before height integration and feedforward. Zero disables smoothing.
    // @Units: s
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("HRTC", 4, AP_CustomControl_LQI, _height_rate_tc, 0.0f),
    // @Param: RMAX
    // @DisplayName: LQI heading rate limit
    // @Description: Maximum heading rate reference. This browser model is qualified through 35 deg/s; not a flight-hardware qualification.
    // @Units: deg/s
    // @Range: 1 35
    // @User: Advanced
    AP_GROUPINFO("RMAX", 5, AP_CustomControl_LQI, _turn_max, 10.0f),
    // @Param: TMAP
    // @DisplayName: Research feasible turn trim map
    // @Description: Use constrained nonlinear trim references and feasible speed limits. Research model only.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("TMAP", 6, AP_CustomControl_LQI, _trim_map, 0),
    // @Param: AWMODE
    // @DisplayName: Research anti-windup allocation
    // @Description: Select full inverse or dominant-channel back calculation for research comparisons.
    // @Values: 0:Full inverse,1:Dominant channel
    // @User: Advanced
    AP_GROUPINFO("AWMODE", 7, AP_CustomControl_LQI, _antiwindup_mode, 0),
    // @Param: OBSTRIM
    // @DisplayName: Research observer equilibrium correction
    // @Description: Recenter the fixed observer dynamics on the nonlinear turn equilibrium.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("OBSTRIM", 8, AP_CustomControl_LQI, _observer_trim, 0),
    AP_GROUPEND
};

AP_CustomControl_LQI::AP_CustomControl_LQI(AP_CustomControl &frontend, float dt) :
    AP_CustomControl_Backend(frontend, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_CustomControl_LQI::valid(const Input &in) const
{
    const auto &s = in.state;
    return isfinite(_turn_max.get()) && _turn_max >= 1 && _turn_max <= 35 &&
        isfinite(_height_rate_tc.get()) && _height_rate_tc >= 0 && _height_rate_tc <= 3 &&
        isfinite(_height_up.get()) && _height_up >= 0.1f && _height_up <= 3.0f &&
        isfinite(_height_down.get()) && _height_down >= 0.1f && _height_down <= 3.0f &&
        isfinite(_height_tc.get()) && _height_tc >= 0.2f && _height_tc <= 10.0f &&
        in.eligible && in.reference.valid && s.attitude_valid && s.navigation_valid &&
        s.airspeed_valid && s.wind_valid && in.time_ms-s.time_ms <= 100 &&
        in.time_ms-in.reference.time_ms <= 500 && isfinite(in.dt) && in.dt > 0 && in.dt <= 0.1f &&
        isfinite(s.attitude_body_to_ned.length_squared()) &&
        fabsf(s.attitude_body_to_ned.length_squared()-1) < 0.01f &&
        !s.velocity_ned_mps.is_nan() && !s.velocity_ned_mps.is_inf() &&
        !s.body_rate_rads.is_nan() && !s.body_rate_rads.is_inf() &&
        !s.wind_ned_mps.is_nan() && !s.wind_ned_mps.is_inf() &&
        isfinite(s.height_home_m) && isfinite(s.true_airspeed_mps) &&
        isfinite(s.eas_to_tas) && s.eas_to_tas > 0 &&
        isfinite(in.reference.climb_rate_mps) &&
        isfinite(in.reference.height_home_m) && isfinite(in.reference.heading_rate_rads) &&
        isfinite(in.reference.equivalent_airspeed_mps);
}

bool AP_CustomControl_LQI::can_run()
{
    return valid(_frontend.mimo_input);
}

void AP_CustomControl_LQI::measurements(const Input &in, float (&y)[9]) const
{
    float roll, pitch, yaw;
    in.state.attitude_body_to_ned.to_euler(roll, pitch, yaw);
    Matrix3f rotation;
    in.state.attitude_body_to_ned.rotation_matrix(rotation);
    const Vector3f air_body = rotation.transposed() * (in.state.velocity_ned_mps-in.state.wind_ned_mps);
    y[0] = (roll-EULER[0])/XS[0];
    y[1] = (pitch-EULER[1])/XS[1];
    for (uint8_t j = 0; j < 3; j++) {
        y[2+j] = (air_body[j]-VELOCITY[j])/XS[5+j];
        y[5+j] = (in.state.body_rate_rads[j]-RATES[j])/XS[8+j];
    }
    y[8] = (in.state.height_home_m-_height_origin)/XS[15];
}

bool AP_CustomControl_LQI::engage(const Input &in)
{
    if (!valid(in) || !in.previous.applied.finite()) {
        return false;
    }
    memset(_state, 0, sizeof(_state));
    memset(_integral, 0, sizeof(_integral));
    _height_origin = in.state.height_home_m;
    _height_reference = 0;
    _smoothed_climb = 0;
    _turn_reference = 0;
    _elapsed = 0;
    _handover_time = 0;
    _reset_sequence = in.state.reset_sequence;
    float y[9];
    measurements(in, y);
    const uint8_t indices[9] = {0, 1, 5, 6, 7, 8, 9, 10, 15};
    for (uint8_t j = 0; j < 9; j++) {
        _state[indices[j]] = y[j];
    }
    if (_trim_map) {
        float x[16],u[3];
        turn_trim(0,in.reference.equivalent_airspeed_mps*in.state.eas_to_tas,x,u);
        const uint8_t hidden[7]={2,3,4,11,12,13,14};
        for (uint8_t j=0; j<7; j++) { _state[hidden[j]]=x[hidden[j]]; }
    }
    _held = in.previous.applied;
    _applied[0] = _held.throttle;
    _applied[1] = _held.common_brake;
    _applied[2] = _held.differential_brake;
    for (uint8_t j=0; j<3; j++) { _handover_start[j]=_applied[j]; }
    for (uint8_t j = 0; j < 3; j++) {
        float feedback = 0;
        for (uint8_t k = 0; k < 16; k++) {
            feedback += K[j][k]*_state[k];
        }
        _matching[j] = _applied[j]-TRIM[j]+US[j]*feedback;
        if (_trim_map) {
            float x[16],u[3];
            turn_trim(0,in.reference.equivalent_airspeed_mps*in.state.eas_to_tas,x,u);
            feedback=0;
            for (uint8_t k=0; k<16; k++) { feedback+=K[j][k]*(_state[k]-x[k]); }
            _matching[j]=_applied[j]-u[j]+US[j]*feedback;
        }
        _requested[j] = _applied[j];
    }
    return true;
}

bool AP_CustomControl_LQI::update_mimo(const Input &in, Demand &demand)
{
    if (!valid(in) || in.state.reset_sequence != _reset_sequence) {
        return false;
    }
    _elapsed += in.dt;
    if (_elapsed < DT) {
        demand = _held;
        return true;
    }
    // Fixed-period matrices: do not integrate a backlog after a scheduling stall.
    if (_elapsed > 2*DT) {
        return false;
    }
    _elapsed -= DT;
    _handover_time = MIN(_handover_time+DT, 2.0f);
    float observer_x[16]{}, observer_u[3]{};
    if (_observer_trim) {
        float r,p,y; in.state.attitude_body_to_ned.to_euler(r,p,y);
        const float turn=(in.state.body_rate_rads.y*sinf(r)+in.state.body_rate_rads.z*cosf(r))/MAX(cosf(p),0.3f);
        turn_trim(degrees(turn),in.state.true_airspeed_mps,observer_x,observer_u);
    }
    float predicted[16]{};
    for (uint8_t i = 0; i < 16; i++) {
        for (uint8_t j = 0; j < 16; j++) {
            predicted[i] += A[i][j]*(_state[j]-(_observer_trim ? observer_x[j] : 0));
        }
        for (uint8_t j = 0; j < 3; j++) {
            predicted[i] += B[i][j]*(_applied[j]-(_observer_trim ? observer_u[j] : TRIM[j]))/US[j];
        }
    }
    if (_observer_trim) { for (uint8_t i=0; i<16; i++) { predicted[i]+=observer_x[i]; } }
    float y[9];
    measurements(in, y);
    const uint8_t indices[9] = {0, 1, 5, 6, 7, 8, 9, 10, 15};
    for (uint8_t i = 0; i < 16; i++) {
        _state[i] = predicted[i];
        for (uint8_t j = 0; j < 9; j++) {
            _state[i] += L[i][j]*(y[j]-predicted[indices[j]]);
        }
        if (!isfinite(_state[i])) {
            return false;
        }
    }
    const float climb = in.reference.vertical == VerticalReference::CLIMB_RATE ?
        in.reference.climb_rate_mps :
        (in.reference.height_home_m-_height_origin-_height_reference)/_height_tc;
    const float requested_climb = constrain_float(climb, -_height_down.get(), _height_up.get());
    _smoothed_climb += DT/(_height_rate_tc+DT)*(requested_climb-_smoothed_climb);
    const float governed_climb = _smoothed_climb;
    _height_reference += DT*governed_climb;
    const float turn = constrain_float(in.reference.heading_rate_rads, -radians(_turn_max), radians(_turn_max));
    _turn_reference += constrain_float(turn-_turn_reference, -radians(40)*DT, radians(40)*DT);
    float roll, pitch, yaw;
    in.state.attitude_body_to_ned.to_euler(roll, pitch, yaw);
    const float heading_rate = (in.state.body_rate_rads.y*sinf(roll)+in.state.body_rate_rads.z*cosf(roll))/MAX(cosf(pitch), 0.3f);
    // The fixed model has a limited speed envelope; faster demands cannot be
    // met by releasing already fully released brakes.
    float speed_target = constrain_float(in.reference.equivalent_airspeed_mps*in.state.eas_to_tas, 3.8f, 4.4f);
    float trim_x[16]{}, trim_u[3]{};
    if (_trim_map) { speed_target=turn_trim(degrees(_turn_reference),speed_target,trim_x,trim_u); }
    const float errors[3] = {
        _state[15]-_height_reference,
        (heading_rate-_turn_reference)/radians(10),
        (in.state.true_airspeed_mps-speed_target)/0.5f
    };
    // Map the entire allocation residual back through the integral gain matrix.
    // Saturating common brake must not freeze the independent turn integral.
    for (uint8_t j = 0; j < 3; j++) {
        float correction = 0;
        for (uint8_t i = 0; i < 3; i++) {
            correction += AW[j][i]*(_requested[i]-_applied[i]);
        }
        if (_antiwindup_mode) {
            // Preserve turn integration when only common brake clips. Each
            // correction uses the dominant actuator and its actual integral gain.
            const uint8_t actuator[3]={0,2,1};
            const uint8_t i=actuator[j];
            correction=(_requested[i]-_applied[i])/(US[i]*K[i][16+j]);
        }
        _integral[j] = constrain_float(_integral[j]+DT*(errors[j]+correction), -50, 50);
    }
    const float references[4] = {_height_reference, _turn_reference/radians(10),
                                (speed_target-Vector3f(VELOCITY[0], VELOCITY[1], VELOCITY[2]).length())/0.5f,
                                governed_climb};
    float target_state[16]{};
    for (uint8_t j = 0; j < 16; j++) {
        for (uint8_t r = 0; r < 4; r++) {
            target_state[j] += XREF[j][r]*references[r];
        }
    }
    if (_trim_map) {
        for (uint8_t j=0; j<16; j++) { target_state[j]=trim_x[j]+XREF[j][0]*references[0]+XREF[j][3]*references[3]; }
    }
    for (uint8_t i = 0; i < 3; i++) {
        float feedback = 0;
        for (uint8_t j = 0; j < 16; j++) {
            feedback += K[i][j]*(_state[j]-target_state[j]);
        }
        for (uint8_t j = 0; j < 3; j++) {
            feedback += K[i][16+j]*_integral[j];
        }
        float feedforward = 0;
        for (uint8_t r = 0; r < 4; r++) {
            feedforward += UREF[i][r]*references[r];
        }
        if (_trim_map) { feedforward=(trim_u[i]-TRIM[i])/US[i]+UREF[i][3]*references[3]; }
        _requested[i] = TRIM[i]+US[i]*(feedforward-feedback)+_matching[i];
        // Let the hidden-state observer settle after a mode/controller handover.
        // Actual allocated outputs continue to drive the observer throughout.
        const float fraction = _handover_time/2.0f;
        const float blend = fraction*fraction*(3.0f-2.0f*fraction);
        _requested[i] = _handover_start[i]+blend*(_requested[i]-_handover_start[i]);
        _matching[i] *= 0.5f/(0.5f+DT);
    }
    _held = {_requested[2], _requested[1], _requested[0]};
    demand = _held;
    return demand.finite();
}

void AP_CustomControl_LQI::allocation_feedback(const Allocation &allocation)
{
    _applied[0] = allocation.applied.throttle;
    _applied[1] = allocation.applied.common_brake;
    _applied[2] = allocation.applied.differential_brake;
}
#endif
