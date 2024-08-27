/*
 * @file thrust_map.cpp
 *
 * Most ESC/motor/propeller combinations provide a highly non-linear map from
 * input to thrust. For the thrust table defined in thrust_map.hpp, this provides
 * the function to translate a desired normalized thrust (0-1) to the necessary
 * input (also 0-1).
 **/


#include "thrust_map.hpp"

static double* signal;
static double* thrust;
static int points;

// Generic linear mapping
static const int linear_map_points = 11;
static double linear_map[][2] = \
{{0.0,	0.0000}, \
 {0.1,	0.1000}, \
 {0.2,	0.2000}, \
 {0.3,	0.3000}, \
 {0.4,	0.4000}, \
 {0.5,	0.5000}, \
 {0.6,	0.6000}, \
 {0.7,	0.7000}, \
 {0.8,	0.8000}, \
 {0.9,	0.9000}, \
 {1.0,	1.0000}};
 
/*
// Crazepony 4pcs EMAX RS2205 2600KV motors with 3-blade DALPROP T5045C props 
// 3 Cell 11.1V nominal
// Crazepony 4pcs BLHeli_32 35A ESC
// only estimates, have to redo with a proper test stand, max thrust is a guess. JK
static const int RS2205_2600KV_3S_points = 21;
static double RS2205_2600KV_3S_map[][2] = \
{{0.00,	0.000000}, \
 {0.05,	1.88}, \
 {0.10, 9.1}, \
 {0.15,	22.8}, \
 {0.20,	41.0}, \
 {0.25,	65.0}, \
 {0.30,	92.5}, \
 {0.35,	122.3}, \
 {0.40,	157.8}, \
 {0.45,	190.0}, \
 {0.50,	213.3}, \
 {0.55,	269.4}, \
 {0.60,	309.0}, \
 {0.65,	348.0}, \
 {0.70,	389.0}, \
 {0.75,	468.0}, \
 {0.80,	504.0}, \
 {0.85,	552.0}, \
 {0.90,	560.0}, \
 {0.95,	607.0}, \
 {1.00,	620.0}};
 */

// Crazepony 4pcs EMAX RS2205 2600KV motors with 3-blade DALPROP T5045C props 
// 3 Cell 11.1V nominal
// Crazepony 4pcs BLHeli_32 35A ESC
// Thrust is in kgf
static const int RS2205_2600KV_3S_points = 21;
static double RS2205_2600KV_3S_map[][2] = \
{{0.00,	0.000000000}, \
 {0.05, 0.002416855}, \
 {0.10, 0.010141865}, \
 {0.15, 0.023368988}, \
 {0.20, 0.041479545}, \
 {0.25, 0.063916887}, \
 {0.30, 0.090126023}, \
 {0.35, 0.115643753}, \
 {0.40, 0.145875234}, \
 {0.45, 0.175643767}, \
 {0.50, 0.203578921}, \
 {0.55, 0.238488956}, \
 {0.60, 0.270053218}, \
 {0.65, 0.30570036}, \
 {0.70, 0.343448215}, \
 {0.75, 0.387672454}, \
 {0.80, 0.431374777}, \
 {0.85, 0.473066881}, \
 {0.90, 0.519530888}, \
 {0.95, 0.55819935}, \
 {1.00, 0.59540747}};

// READYTOSKY 4pcs RS2212 920KV motors with 2-blade 1045 props
// 4 Cell 14.8V nominal 60C 6500mAh (measured at full charge 16.6V)
// Crazepony 4pcs BLHeli_32 35A ESC
// Thrust is in kgf
static const int RS2212_920_4S_points = 21;
static double RS2212_920_4S_map[][2] = \
{{0.00,	0.000000000}, \
 {0.05, 0.004942223}, \
 {0.10, 0.0240651}, \
 {0.15, 0.05298061}, \
 {0.20, 0.089153005}, \
 {0.25, 0.135371615}, \
 {0.30, 0.185832035}, \
 {0.35, 0.245638744}, \
 {0.40, 0.309358441}, \
 {0.45, 0.382884378}, \
 {0.50, 0.451223411}, \
 {0.55, 0.530646538}, \
 {0.60, 0.597298111}, \
 {0.65, 0.662321926}, \
 {0.70, 0.724290841}, \
 {0.75, 0.78653925}, \
 {0.80, 0.861880171}, \
 {0.85, 0.931260177}, \
 {0.90, 1.01215949}, \
 {0.95, 1.067049587}, \
 {1.00, 1.119747581}};

// READYTOSKY 4pcs RS2212 920KV motors with 2-blade 1045 props
// 3 Cell 11.1V nominal 30C 3000mAh (measured at full charge 12.6V)
// Crazepony 4pcs BLHeli_32 35A ESC
// Thrust is in kgf
static const int RS2212_920_3S_points = 21;
static double RS2212_920_3S_map[][2] = \
{{0.00,	0.000000000}, \
 {0.05, 0.0032067}, \
 {0.10, 0.014764471}, \
 {0.15, 0.033218274}, \
 {0.20, 0.055452587}, \
 {0.25, 0.083547513}, \
 {0.30, 0.115633259}, \
 {0.35, 0.153118916}, \
 {0.40, 0.190474706}, \
 {0.45, 0.235728455}, \
 {0.50, 0.281291187}, \
 {0.55, 0.327476634}, \
 {0.60, 0.37204629}, \
 {0.65, 0.421345197}, \
 {0.70, 0.477803467}, \
 {0.75, 0.534005217}, \
 {0.80, 0.584730816}, \
 {0.85, 0.634994969}, \
 {0.90, 0.687778781}, \
 {0.95, 0.734884904}, \
 {1.00, 0.77382038}};


// Tiger Motor MN1806, 1400KV 6x4.5" 3-blade prop, 14.8V,
// BLheli ESC Low Timing
// this one is in Newtons but it doesn't really matter
static const int mn1806_1400kv_4s_points = 11;
static double mn1806_1400kv_4s_map[][2] = \
{{0.0,	0.0000}, \
 {0.1,	0.2982}, \
 {0.2,	0.6310}, \
 {0.3,	1.0281}, \
 {0.4,	1.5224}, \
 {0.5,	2.0310}, \
 {0.6,	2.5791}, \
 {0.7,	3.1365}, \
 {0.8,	3.7282}, \
 {0.9,	4.3147}, \
 {1.0,	4.7258}};


 // motor used in butterfly project
static const int team2_motor_and_prop_points = 11;
static double team2_motor_and_prop_map[][2] = \
{{0.0,	0.000000000}, \
{0.1,	0.104363983}, \
{0.2,	0.365476752}, \
{0.3,	0.708754533}, \
{0.4,	1.07956102}, \
{0.5,	1.54929885}, \
{0.6,	2.199608144}, \
{0.7,	3.024183382}, \
{0.8,	3.884546001}, \
{0.9,	4.756770189}, \
{1.0,	5.268364677}};

// tiger motor F20 2300kv motor, 2S lipo, 4x4.0" 3-blade props
// blheli esc med-low timing
// thrust units in gram-force but doesn't really matter
static const int f20_2300kv_2s_points = 21;
static double f20_2300kv_2s_map[][2] = \
{{0.00,	0.000000}, \
 {0.05,	6.892067}, \
 {0.10,	12.57954}, \
 {0.15,	18.84790}, \
 {0.20,	26.16294}, \
 {0.25,	33.98255}, \
 {0.30,	41.60790}, \
 {0.35,	49.32732}, \
 {0.40,	58.27048}, \
 {0.45,	67.83613}, \
 {0.50,	78.20817}, \
 {0.55,	88.27728}, \
 {0.60,	100.1058}, \
 {0.65,	110.3643}, \
 {0.70,	121.6316}, \
 {0.75,	132.2155}, \
 {0.80,	145.0420}, \
 {0.85,	154.6838}, \
 {0.90,	162.0185}, \
 {0.95,	168.4321}, \
 {1.00,	177.1643}};



/*
 * Lumenier RX2206-13 2000kv motor, 4S lipo, 5x45" lumenier prop
 * blheli esc high timing
 * for 5" monocoque hex
 */
static const int rx2206_4s_points = 12;
static double rx2206_4s_map[][2] = \
{{0.0	,	0.00000000000000}, \
 {0.05	,	17.8844719758775}, \
 {0.145	,	44.8761484808831}, \
 {0.24	,	80.0271164157384}, \
 {0.335	,	122.556484678150}, \
 {0.43	,	168.358712108506}, \
 {0.525	,	220.433636910433}, \
 {0.62	,	277.201919870206}, \
 {0.715	,	339.008615108196}, \
 {0.81	,	418.819295994349}, \
 {0.905	,	505.430124336786}, \
 {1.0	,	566.758535098236}};


int thrust_map_init(thrust_map_t map)
{
	int i;
	double max;
	double (*data)[2]; // pointer to constant data

	switch(map){

	case TEAM2_PROP:
		points = team2_motor_and_prop_points;
		data = team2_motor_and_prop_map;
		break;
	case RS2205_2600KV_3S:
		points = RS2205_2600KV_3S_points;
		data = RS2205_2600KV_3S_map;
		break;
    case RS2212_920_4S:
        points = RS2212_920_4S_points;
        data = RS2212_920_4S_map;
        break;
    case RS2212_920_3S:
        points = RS2212_920_3S_points;
        data = RS2212_920_3S_map;
        break;
	case LINEAR_MAP:
		points = linear_map_points;
		data = linear_map;
		break;
	case MN1806_1400KV_4S:
		points = mn1806_1400kv_4s_points;
		data = mn1806_1400kv_4s_map;
		break;
	case F20_2300KV_2S:
		points = f20_2300kv_2s_points;
		data = f20_2300kv_2s_map;
		break;
	case RX2206_4S:
		points = rx2206_4s_points;
		data = rx2206_4s_map;
		break;
	default:
		fprintf(stderr,"ERROR: unknown thrust map\n");
		return -1;
	}

	// sanity checks
	if(points<2){
		fprintf(stderr,"ERROR: need at least 2 datapoints in THRUST_MAP\n");
		return -1;
	}
	if(data[0][0] != 0.0){
		fprintf(stderr,"ERROR: first row input must be 0.0\n");
		return -1;
	}
	if(data[points-1][0] != 1.0){
		fprintf(stderr,"ERROR: last row input must be 1.0\n");
		printf("data: %f\n",data[points-1][0]);
		return -1;
	}
	if(data[0][1] != 0.0){
		fprintf(stderr,"ERROR: first row thrust must be 0.0\n");
		return -1;
	}
	if(data[points-1][1] < 0.0){
		fprintf(stderr,"ERROR: last row thrust must be > 0.0\n");
		return -1;
	}
	for(i=1;i<points;i++){
		if(data[i][0]<=data[i-1][0] || data[i][1]<=data[i-1][1]){
			fprintf(stderr,"ERROR: thrust_map must be monotonically increasing\n");
			return -1;
		}
	}

	// create new global array of normalized thrust and inputs
	if(signal!=NULL) free(signal);
	if(thrust!=NULL) free(thrust);
	signal = (double*) malloc(points * sizeof(double));
	thrust = (double*) malloc(points * sizeof(double));
	max = data[points-1][1];
	for(i=0; i<points; i++){
		signal[i] = data[i][0];
		thrust[i] = data[i][1]/max;
	}
	return 0;
}


double map_motor_signal(double m){
	int i;
	double pos;

	// sanity check
	if(m>1.0 || m<0.0){
		printf("ERROR: desired thrust t must be between 0.0 & 1.0\n");
		return -1;
	}

	// return quickly for boundary conditions
	if(m==0.0 || m==1.0) return m;

	// scan through the data to pick the upper and lower points to interpolate
	for(i=1; i<points; i++){
		if(m <= thrust[i]){
			pos = (m-thrust[i-1])/(thrust[i]-thrust[i-1]);
			return signal[i-1]+(pos*(signal[i]-signal[i-1]));
		}
	}

	fprintf(stderr,"ERROR: something in map_motor_signal went wrong\n");
	return -1;
}