typedef struct pid_data_structure {
	int Kp;  	//precision: .01
	int Ki;  	//precision: .001
	int Kd;		//precision: .1
	int prevError;
	int totalError;
	int Ki_Limit;
	int completion_threshold;
	char loop_done;
} DT_PID;

extern DT_PID left_wheel;
extern DT_PID right_wheel;
extern DT_PID shoulder_enc;

unsigned char pid_control(DT_PID* pid_data, int error);
int pid_control_raw(DT_PID* pid_data, int error);
void init_pid(DT_PID* pid_data, int P, int I, int D, int iRange, int ct);
void pid_set_Kp(DT_PID* pid_data, int value);
char pid_isDone(DT_PID* pid_data);

#define pid_incomplete	0
#define pid_complete	1
#define pid_inRange		2

