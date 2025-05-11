from argparse import ArgumentParser as parser
from pyperclip import copy

# helpers
parse = parser(
	prog="pin to hex",
	description="convert an STM32 pin for a peripheral to hex"
)
flag =			{"action": "store_true"}
input_default =	{"action": "store", "type": int, "default": None}


# constants
clocks = {
	"AHB1": 0,
	"AHB2": 1,
	"AHB3": 2,
	"APB1": 4,
	"APB2": 5
}

ports = {
	"A": 0, "B": 1, "C": 2, "D": 3,
	"E": 4, "F": 5, "G": 6, "H": 7,
	"I": 8, "J": 9, "K": 10
}

tims = {
	"TIM1": ("APB2", 0),    "TIM8": ("APB2", 1),
	"TIM9": ("APB2", 16),	"TIM10": ("APB2", 17),
	"TIM11": ("APB2", 18),	"TIM2": ("APB1", 0),
	"TIM3": ("APB1", 1),	"TIM4": ("APB1", 2),
	"TIM5": ("APB1", 3),	"TIM6": ("APB1", 4),
	"TIM7": ("APB1", 5),	"TIM12": ("APB1", 6),
	"TIM13": ("APB1", 7),	"TIM14": ("APB1", 8),
}
uarts = {
	"UART1": ("APB2", 4),    "UART2": ("APB1", 17),
	"UART3": ("APB1", 18),   "UART6": ("APB2", 5)
}
i2cs = {
	"I2C1": ("APB1", 21),    "I2C2": ("APB1", 22)
}
usbs = {
	"USB1": ("AHB1", 25)
}
spis = {
	"SPI1": ("APB2", 12),	"SPI2": ("APB1", 14),
	"SPI3": ("APB1", 15),	"SPI4": ("APB2", 13),
	"SPI5": ("APB2", 20)
}


# functions
def synthesize(clk: int, dev: int, sub: int, port: int, pin: int, alt: int) -> int:
	return (
		((alt	& 0xf) << 28)	|
		((pin	& 0xf) << 24)	|
		((port	& 0xf) << 20)	|
		((sub	& 0xff) << 12)	|
		((dev	& 0xff) << 4)	|
		((clk	& 0xf) << 0)
	)


def run(type: str, collection: dict, af: int or None, pnum: int or None) -> None:
	while True:
		try:
			sub = 0
			periph = input(f"{type.lower()}: ") if not pnum else pnum
			try:    periph = collection[f"{type}{int(periph)}"]
			except: periph = tims[periph.upper()]
			clk, dev = periph
			clk = clocks[clk]
			if type in ["TIM"]:
				ch = input("channel: ")
				chn = 4 if "n" in ch else 0
				channel = max((int(ch.replace("n", "")) + chn - 1), 0)
				sub |= channel & 0x7
			
			alt = int(input("alt: ")) if not af else af
			pin = input("pin: ")
			port = int(ports[pin[0].upper()])
			pin = int(pin[1:])
			res = synthesize(clk, dev, sub, port, pin, alt)
			msg = f"{res:#0{10}x}UL,\t".upper().replace("X", "x")
			print(msg, end="\n\n")
			copy(msg)
		except KeyboardInterrupt:   exit(0)
		except Exception as e:      print(e); pass


if __name__ == "__main__":
	parse.add_argument('-af', **input_default)
	parse.add_argument('-pnum', **input_default)
	
	parse.add_argument('-tim', **flag)
	parse.add_argument('-uart', **flag)
	parse.add_argument('-i2c', **flag)
	parse.add_argument('-usb', **flag)
	parse.add_argument('-spi', **flag)
	arg = parse.parse_args()
	
	if arg.tim:		run("TIM", tims, arg.af, arg.pnum)
	if arg.uart:	run("UART", uarts, arg.af, arg.pnum)
	if arg.i2c:		run("I2C", i2cs, arg.af, arg.pnum)
	if arg.spi:		run("SPI", spis, arg.af, arg.pnum)
	
