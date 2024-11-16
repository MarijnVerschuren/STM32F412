from re import finditer

text = """
typedef enum {
	I2C_PIN_DISABLE =	0x00000000UL,
	// I2C1
	I2C1_B6_SCL =		0x46100153UL,
	I2C1_B7_SDA =		0x47100153UL,
	I2C1_B8_SCL =		0x48100153UL,
	I2C1_B9_SDA = 		0x49100153UL,
	// I2C2
	I2C2_B9_SDA =		0x99100163UL,
	I2C2_B10_SCL =		0x4A100163UL
} I2C_GPIO_t;
"""

translate = {
	"clk": {
		0: 0,
		1: 1,
		2: 2,
		3: 4,
		4: 5
	}
}

def ifunc(id: str) -> tuple:
	id = int(id[2:-2], 16)
	return (
		((id >> 0) & 0xF),
		((id >> 4) & 0xFF),
		((id >> 12) & 0xFF),
		((id >> 20) & 0xF),
		((id >> 24) & 0xF),
		((id >> 28) & 0xF)
	)

def ofunc(data: tuple) -> str:
	if data == (0, 0, 0, 0, 0, 0): return "0x00000000UL"
	res = (
		((translate["clk"][data[0]] & 0xF) << 0)	|
		((data[1] & 0xFF) << 4)						|
		((data[2] & 0xFF) << 12)					|
		((data[3] & 0xF) << 20)						|
		((data[4] & 0xF) << 24)						|
		((data[5] & 0xF) << 28)
	)
	return f"{res:#0{10}x}UL".upper().replace("X", "x")


if __name__ == "__main__":
	tokens = [(i.start(), i.end()) for i in finditer("0x........UL", text)]
	for ts, te in tokens:
		text = text[:ts] + ofunc(ifunc(text[ts:te])) + text[te:]
	print(text)
