/*
 *  core.h
 *
 */

/* 
 * Macros for performance counters and its control registers 
 * Refer to the Intel Arch Manual
 */
#define IA32_FIXED_CTR_CTRL 0x38D
#define IA32_PERF_GLOBAL_CTRL 0x38F
#define PERF_FIXED_CTR0 0x309
#define PERF_FIXED_CTR1 0x30A
#define PERF_FIXED_CTR2 0x30B
#define PerfEvtSel0 0x186
#define PerfEvtSel1 0x187
#define PerfEvtSel2 0x188
#define PerfEvtSel3 0x189
#define IA32_PMC0 0xC1
#define IA32_PMC1 0xC2
#define IA32_PMC2 0xC3
#define IA32_PMC3 0xC4
#define OFFCORE_RSP_0 0x1A6

#define FC0 PERF_FIXED_CTR0
#define FC1 PERF_FIXED_CTR1
#define FC2 PERF_FIXED_CTR2
#define FCC IA32_FIXED_CTR_CTRL
#define GLC IA32_PERF_GLOBAL_CTRL
#define PCC0 PerfEvtSel0
#define PCC1 PerfEvtSel1
#define PCC2 PerfEvtSel2
#define PCC3 PerfEvtSel3
#define PC0 IA32_PMC0
#define PC1 IA32_PMC1
#define PC2 IA32_PMC2
#define PC3 IA32_PMC3
#define OFF0 OFFCORE_RSP_0
/* 
 * Machine specific information. 
 * Can be exported into config in the future or read
 * in real time
 */

#define CA_I	PERF_FIXED_CTR0
//base
#define CA_BC	PERF_FIXED_CTR1
//stall
#define CA_SC	0x004101a2
//miss
#define CA_MC	0x004104a3



void move_to_cpu(const int cpu, struct task_struct *tsk) {
	struct cpumask tmp_mask;
	cpumask_clear(&tmp_mask);
	cpumask_set_cpu((unsigned int)cpu, &tmp_mask);
	sched_setaffinity(tsk->pid, &tmp_mask);
}

int get_task_cpu(struct task_struct *tsk) {
	return task_cpu(tsk);
}


static void setup_round1(const int cpu)
{
	unsigned int low, high;
	low = 0;
	high = 0;

	//native_write_msr(FC0,0,0);
	native_write_msr(CA_I,0,0);
	//native_write_msr(FC1,0,0);
	native_write_msr(CA_BC,0,0);
	native_write_msr(FC2,0,0);
	native_write_msr(PC0,0,0);
	native_write_msr(PC1,0,0);
	native_write_msr(PC2,0,0);
	native_write_msr(PC3,0,0);

	native_write_msr(PCC0,0x004101a2,0);
	native_write_msr(PCC1,0x004104a3,0);
	//native_write_msr(PCC0,0x004110d1,0);
	//native_write_msr(PCC1,0x004104d1,0);
	native_write_msr(PCC2,0x0041412e,0);
	native_write_msr(PCC3,0x00410cd3,0);
	native_write_msr(FCC,819,0);
	native_write_msr(GLC,15,7);

	return ;
}

static void setup_round2(const int cpu) {
	unsigned int low, high;

	low = 0;
	high = 0;

	native_write_msr(FC0,0,0);
	native_write_msr(FC1,0,0);
	native_write_msr(FC2,0,0);
	native_write_msr(PC0,0,0);
	native_write_msr(PC1,0,0);
	native_write_msr(PC2,0,0);
	native_write_msr(PC3,0,0);

	native_write_msr(PCC0,0x004120d3,0);
	native_write_msr(PCC1,0x004110d3,0);
	native_write_msr(PCC2,0x004120d1,0);
	native_write_msr(PCC3,0x004101b7,0);
	native_write_msr(FCC,819,0);
	native_write_msr(GLC,15,7);
	native_write_msr(OFF0,0xFFC00030,0x3F);
	return ;
}

static void collect_round1(const int cpu, struct task_struct *tsk)
{
	native_write_msr(PCC0,0x000101a2,0);
	native_write_msr(PCC1,0x000104a3,0);
	//native_write_msr(PCC0,0x000110d1,0);
	//native_write_msr(PCC1,0x000104d1,0);
	native_write_msr(PCC2,0x0001412e,0);
	native_write_msr(PCC3,0x00010cd3,0);

	tsk->temp.inst = native_read_msr(FC0);
	tsk->temp.base = native_read_msr(FC1);
	tsk->temp.unhalted = native_read_msr(FC2);
	tsk->temp.unhalted1 = tsk->temp.unhalted;
	tsk->temp.stall = native_read_msr(PC0);
	tsk->temp.miss = native_read_msr(PC1);
	tsk->temp.llc_misses = native_read_msr(PC2);
	tsk->temp.remote_dram = native_read_msr(PC3);

	//tsk->log.unhalted1 += tsk->temp.unhalted1;
	tsk->log.inst += tsk->temp.inst;
	//tsk->log.llc_l2miss += tsk->temp.llc_l2miss;
	tsk->log.base += tsk->temp.base;
	//tsk->log.llc_l3hit += tsk->temp.llc_l3hit;
	tsk->log.stall += tsk->temp.stall;
	tsk->log.llc_misses += tsk->temp.llc_misses;
	//tsk->log.remote_dram += tsk->temp.remote_dram;
	tsk->log.miss += tsk->temp.miss;

	return ;
}

static void collect_round2(const int cpu, struct task_struct *tsk)
{
	native_write_msr(PCC0,0x000120d3,0);
	native_write_msr(PCC1,0x000110d3,0);
	native_write_msr(PCC2,0x000120d1,0);
	native_write_msr(PCC3,0x000101b7,0);

	tsk->temp.inst += native_read_msr(FC0);
	tsk->log.inst += tsk->temp.inst;
	tsk->temp.inst = tsk->temp.inst / 2;
	tsk->temp.cycles += native_read_msr(FC1);
	tsk->log.cycles += tsk->temp.cycles;
	tsk->temp.cycles = tsk->temp.cycles / 2;
	tsk->temp.unhalted2 = native_read_msr(FC2);
	tsk->temp.unhalted += tsk->temp.unhalted2;
	tsk->log.unhalted += tsk->temp.unhalted;
	tsk->log.unhalted2 += tsk->temp.unhalted2;
	tsk->temp.unhalted = tsk->temp.unhalted / 2;
	tsk->temp.llc_remotefwd = native_read_msr(PC0);
	tsk->temp.llc_remotehitm = native_read_msr(PC1);
	tsk->temp.llc_l3miss = native_read_msr(PC2);
	tsk->temp.llc_prefetch = native_read_msr(PC3);
	tsk->log.llc_remotefwd += tsk->temp.llc_remotefwd;
	tsk->log.llc_remotehitm += tsk->temp.llc_remotehitm;
	tsk->log.llc_l3miss += tsk->temp.llc_l3miss;
	tsk->log.llc_prefetch += tsk->temp.llc_prefetch;

	return ;
}

