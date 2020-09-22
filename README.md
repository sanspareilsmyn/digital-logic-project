# digital-logic-project
2019년 2학기 &lt;논리설계 및 실험> 기말 프로젝트입니다.

1. Project name : CPU on FPGA - 16 bit single-cycle CPU
2. Project overview:
Verilog를 이용하여 FPGA Board 상에 16-bit single-cycle CPU를 구현하였다.
디자인과 시뮬레이션 툴에는 Xilinx ISE를, HDL은 Verilog를, FPGA는 Spartan 3을 사용하였다.

![image](https://user-images.githubusercontent.com/52681837/93869784-3c826b80-fd07-11ea-8635-bf418228856d.png)

1) Green Circle 부분은 Memory를 Instruction으로 초기화하는 코드이며, 조교님이 이미 제공하신 코드다.
2) Blue Circle은 7개의 모듈이며, CPU 모듈과 연결된다.
3) Red Circle은 간단한 명령(concatenation, sign extension)이므로 굳이 모듈로 만들지 않고 다른 모듈에 포함시킨다.

3. Implementation
CPU 안에 8개의 submodule이 들어간다. 초기 조건 7개에 1개가 추가됐는데, 이는 output_port이다. reg 값을 출력시킬 때 더 편하게 코드를 짜기 위해 register_memory 모듈을 하나 추가했다.

1) Register

Register 값들을 읽고 쓰는 기능을 담당한다. CPU가 reset되면 각 register를 0으로 초기화시키고, control 모듈에서 RegWrite 신호가 1로 오면 새 값을 register에 쓴다. r0 ~ r3는 각 register 값을 output_port로 보내기 위한 output이다.

2) reg_MUX

control 모듈에서 오는 RegDst 값에 따라 Instruction의 [9:8] 또는 [7:6] 중 하나를 register 모듈의 write_reg로 보낸다.

3) register_memory

register 모듈에서 write한 data를 output_port에 전달한다. input으로는 clk, rst, wwd_enable, instruction, register_selection, 그리고 r0 ~ r3를 받는다. output은 output_port와 연결하기 위해 하나 존재한다. cpu가 reset되면 0000을 출력하고, wwd instruction과 wwd_enable을 확인하여 출력되는 reg 번호를 판단해 출력한다.

4) control

Instruction의 앞 4자리(opcode)와 뒷 6자리(function)을 받아서 5개의 모듈로 신호를 보내 cpu가 수행해야 할 동작을 제어한다. ADD, ADI, LHI, JMP, WWD를 case문으로 처리한다.

5) JMP_MUX

Jump 신호에 따라 Instruction 끝 12자리인 target과 현재 pc의 앞 4자리를 이어 jump_pc를 만들고 이를 CPU 모듈로 보낸다.

6) ALU_Control

ALUOp 신호를 보고 JMP/WWD면 ALU를 끄고, 아니면 ALU_func에 1을 보내 ALU를 켠다.

7) ALU_MUX

ALUScr가 0이면 ADD나 WWD기 때문에 read_data2를 ALU의 2nd input으로 넣는다. 그렇지 않으면 ADI나 LHI기 때문에 sign extension 또는 뒤에 0을 8개 붙여서 ALU의 2nd input으로 넣는다.

8) ALU

ALU_MUX에서 오는 ALU_Control이 1이면 동작한다. opcode를 보고 LHI면 2nd input을 register의 write_data로 보내고, ADD/ADI/WWD면 input 2개를 더해서 write_data로 보낸다.
