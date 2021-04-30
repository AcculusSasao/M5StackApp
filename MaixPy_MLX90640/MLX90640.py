## MaixPy MLX90640 sensor driver via i2c.
## Copyright (c) 2021 Yukiyoshi Sasao
## M5Stack C++ example: https://github.com/m5stack/M5Stack/tree/master/examples/Unit/THERMAL_MLX90640
##   Copyright (c) 2017 M5Stack : MIT License : https://github.com/m5stack/M5Stack/blob/master/LICENSE
## MLX90640 datasheet: https://www.melexis.com/en/documents/documentation/datasheets/datasheet-mlx90640

from machine import I2C
import struct, math, array, image, sensor

# parameter parser from EEPROM data
class MLX90640param:
	SCALEALPHA = 0.000001
	def __init__(self, eeData, COLS, ROWS):
		# ExtractVDDParameters
		kVdd = (eeData[51] & 0xFF00) >> 8
		if kVdd > 127:
			kVdd = kVdd - 256
		kVdd = 32 * kVdd
		vdd25 = eeData[51] & 0x00FF
		vdd25 = ((vdd25 - 256) << 5) - 8192
		self.kVdd = kVdd
		self.vdd25 = vdd25
		# ExtractPTATParameters
		KvPTAT = (eeData[50] & 0xFC00) >> 10
		if KvPTAT > 31:
			KvPTAT = KvPTAT - 64
		KvPTAT = KvPTAT/4096
		KtPTAT = eeData[50] & 0x03FF
		if KtPTAT > 511:
			KtPTAT = KtPTAT - 1024
		KtPTAT = KtPTAT/8
		vPTAT25 = eeData[49]
		alphaPTAT = (eeData[16] & 0xF000) / 16384 + 8.0		
		self.KvPTAT = KvPTAT
		self.KtPTAT = KtPTAT    
		self.vPTAT25 = vPTAT25
		self.alphaPTAT = alphaPTAT
		# ExtractGainParameters
		gainEE = eeData[48]
		if gainEE > 32767:
			gainEE = gainEE -65536
		self.gainEE = gainEE
		# ExtractTgcParameters
		tgc = eeData[60] & 0x00FF
		if tgc > 127:
			tgc = tgc - 256
		tgc = tgc / 32.0
		self.tgc = tgc
		# ExtractResolutionParameters
		resolutionEE = (eeData[56] & 0x3000) >> 12
		self.resolutionEE = resolutionEE
		# ExtractKsTaParameters
		KsTa = (eeData[60] & 0xFF00) >> 8
		if KsTa > 127:
			KsTa = KsTa -256
		KsTa = KsTa / 8192.0
		self.KsTa = KsTa
		# ExtractKsToParameters
		step = ((eeData[63] & 0x3000) >> 12) * 10
		self.ct = [-40, 0, 0, 0, 0]
		self.ct[2] = (eeData[63] & 0x00F0) >> 4
		self.ct[3] = (eeData[63] & 0x0F00) >> 8
		self.ct[2] = self.ct[2]*step
		self.ct[3] = self.ct[2] + self.ct[3]*step
		self.ct[4] = 400
		KsToScale = (eeData[63] & 0x000F) + 8
		KsToScale = 1 << KsToScale
		self.ksTo = [0,0,0,0,0]
		self.ksTo[0] = eeData[61] & 0x00FF
		self.ksTo[1] = (eeData[61] & 0xFF00) >> 8
		self.ksTo[2] = eeData[62] & 0x00FF
		self.ksTo[3] = (eeData[62] & 0xFF00) >> 8
		for i in range(4):
			if self.ksTo[i] > 127:
				self.ksTo[i] = self.ksTo[i] - 256
			self.ksTo[i] = self.ksTo[i] / KsToScale
		self.ksTo[4] = -0.0002
		# ExtractCPParameters
		alphaSP = [0]*2
		offsetSP = [0]*2
		alphaScale = ((eeData[32] & 0xF000) >> 12) + 27		
		offsetSP[0] = (eeData[58] & 0x03FF)
		if offsetSP[0] > 511:
			offsetSP[0] = offsetSP[0] - 1024
		offsetSP[1] = (eeData[58] & 0xFC00) >> 10
		if offsetSP[1] > 31:
			offsetSP[1] = offsetSP[1] - 64
		offsetSP[1] = offsetSP[1] + offsetSP[0]		
		alphaSP[0] = (eeData[57] & 0x03FF)
		if alphaSP[0] > 511:
			alphaSP[0] = alphaSP[0] - 1024
		alphaSP[0] = alphaSP[0] /  (2**alphaScale)
		alphaSP[1] = (eeData[57] & 0xFC00) >> 10
		if alphaSP[1] > 31:
			alphaSP[1] = alphaSP[1] - 64
		alphaSP[1] = (1 + alphaSP[1]/128) * alphaSP[0]		
		cpKta = (eeData[59] & 0x00FF)
		if cpKta > 127:
			cpKta = cpKta - 256
		ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8
		self.cpKta = cpKta / (2**ktaScale1)
		cpKv = (eeData[59] & 0xFF00) >> 8
		if cpKv > 127:
			cpKv = cpKv - 256
		kvScale = (eeData[56] & 0x0F00) >> 8
		self.cpKv = cpKv / (2**kvScale)
		self.cpAlpha = [ alphaSP[0], alphaSP[1] ]
		self.cpOffset = [ offsetSP[0], offsetSP[1] ]
		# ExtractAlphaParameters
		accRow = [0]*24
		accColumn = [0]*32
		alphaTemp = [0]*(COLS*ROWS)
		accRemScale = eeData[32] & 0x000F
		accColumnScale = (eeData[32] & 0x00F0) >> 4
		accRowScale = (eeData[32] & 0x0F00) >> 8
		alphaScale = ((eeData[32] & 0xF000) >> 12) + 30
		alphaRef = eeData[33]
		for i in range(6):
			p = i * 4
			accRow[p + 0] = (eeData[34 + i] & 0x000F)
			accRow[p + 1] = (eeData[34 + i] & 0x00F0) >> 4
			accRow[p + 2] = (eeData[34 + i] & 0x0F00) >> 8
			accRow[p + 3] = (eeData[34 + i] & 0xF000) >> 12
		for i in range(24):
			if accRow[i] > 7:
				accRow[i] = accRow[i] - 16
		for i in range(8):
			p = i * 4
			accColumn[p + 0] = (eeData[40 + i] & 0x000F)
			accColumn[p + 1] = (eeData[40 + i] & 0x00F0) >> 4
			accColumn[p + 2] = (eeData[40 + i] & 0x0F00) >> 8
			accColumn[p + 3] = (eeData[40 + i] & 0xF000) >> 12
		for i in range(32):
			if accColumn[i] > 7:
				accColumn[i] = accColumn[i] - 16
		for i in range(24):
			for j in range(32):
				p = 32 * i +j
				alphaTemp[p] = (eeData[64 + p] & 0x03F0) >> 4
				if alphaTemp[p] > 31:
					alphaTemp[p] = alphaTemp[p] - 64
				alphaTemp[p] = alphaTemp[p]*(1 << accRemScale)
				alphaTemp[p] = (alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) + alphaTemp[p])
				alphaTemp[p] = alphaTemp[p] / (2**alphaScale)
				alphaTemp[p] = alphaTemp[p] - self.tgc * (self.cpAlpha[0] + self.cpAlpha[1])/2
				alphaTemp[p] = self.SCALEALPHA/alphaTemp[p]
		temp = alphaTemp[0]
		for i in range(1,768):
			if alphaTemp[i] > temp:
				temp = alphaTemp[i]
		alphaScale = 0
		while temp < 32768:
			temp = temp*2
			alphaScale = alphaScale + 1
		self.alpha = array.array('H',[0]*(COLS*ROWS))
		for i in range(0,768):
			temp = alphaTemp[i] * (2**alphaScale)
			self.alpha[i] = int(temp + 0.5)
		self.alphaScale = alphaScale
		del accRow
		del accColumn
		del alphaTemp
		# ExtractOffsetParameters
		occRow = [0]*24
		occColumn = [0]*32
		occRemScale = (eeData[16] & 0x000F)
		occColumnScale = (eeData[16] & 0x00F0) >> 4
		occRowScale = (eeData[16] & 0x0F00) >> 8
		offsetRef = eeData[17]
		if offsetRef > 32767:
			offsetRef = offsetRef - 65536
		for i in range(6):
			p = i * 4
			occRow[p + 0] = (eeData[18 + i] & 0x000F)
			occRow[p + 1] = (eeData[18 + i] & 0x00F0) >> 4
			occRow[p + 2] = (eeData[18 + i] & 0x0F00) >> 8
			occRow[p + 3] = (eeData[18 + i] & 0xF000) >> 12
		for i in range(24):
			if occRow[i] > 7:
				occRow[i] = occRow[i] - 16
		for i in range(8):
			p = i * 4
			occColumn[p + 0] = (eeData[24 + i] & 0x000F)
			occColumn[p + 1] = (eeData[24 + i] & 0x00F0) >> 4
			occColumn[p + 2] = (eeData[24 + i] & 0x0F00) >> 8
			occColumn[p + 3] = (eeData[24 + i] & 0xF000) >> 12
		for i in range(32):
			if occColumn[i] > 7:
				occColumn[i] = occColumn[i] - 16
		self.offset = array.array( 'h', [0]*(COLS*ROWS) )
		for i in range(24):
			for j in range(32):
				p = 32 * i +j
				self.offset[p] = (eeData[64 + p] & 0xFC00) >> 10
				if self.offset[p] > 31:
					self.offset[p] = self.offset[p] - 64
				self.offset[p] = self.offset[p]*(1 << occRemScale)
				self.offset[p] = (offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + self.offset[p])
		del occRow
		del occColumn
		# ExtractKtaPixelParameters
		KtaRC = [0]*4
		ktaTemp = [0]*(COLS*ROWS)
		KtaRoCo = (eeData[54] & 0xFF00) >> 8
		if KtaRoCo > 127:
			KtaRoCo = KtaRoCo - 256
		KtaRC[0] = KtaRoCo
		KtaReCo = (eeData[54] & 0x00FF)
		if KtaReCo > 127:
			KtaReCo = KtaReCo - 256
		KtaRC[2] = KtaReCo
		KtaRoCe = (eeData[55] & 0xFF00) >> 8
		if KtaRoCe > 127:
			KtaRoCe = KtaRoCe - 256
		KtaRC[1] = KtaRoCe
		KtaReCe = (eeData[55] & 0x00FF)
		if KtaReCe > 127:
			KtaReCe = KtaReCe - 256
		KtaRC[3] = KtaReCe
		ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8
		ktaScale2 = (eeData[56] & 0x000F)
		for i in range(24):
			for j in range(32):
				p = 32 * i +j
				split = 2*(p//32 - (p//64)*2) + p%2
				ktaTemp[p] = (eeData[64 + p] & 0x000E) >> 1
				if ktaTemp[p] > 3:
					ktaTemp[p] = ktaTemp[p] - 8
				ktaTemp[p] = ktaTemp[p] * (1 << ktaScale2)
				ktaTemp[p] = KtaRC[split] + ktaTemp[p]
				ktaTemp[p] = ktaTemp[p] / (2**ktaScale1)
		temp = abs(ktaTemp[0])
		for i in range(1,768):
			if abs(ktaTemp[i]) > temp:
				temp = abs(ktaTemp[i])
		ktaScale1 = 0
		while temp < 64:
			temp = temp*2
			ktaScale1 = ktaScale1 + 1
		self.kta = array.array( 'b', [0]*(COLS*ROWS) )
		for i in range(768):
			temp = ktaTemp[i] * (2**ktaScale1)
			if temp < 0:
				self.kta[i] = int(temp - 0.5)
			else:
				self.kta[i] = int(temp + 0.5)
		self.ktaScale = ktaScale1
		del KtaRC
		del ktaTemp
		# ExtractKvPixelParameters
		KvT = [0]*4
		kvTemp = [0]*(COLS*ROWS)
		KvRoCo = (eeData[52] & 0xF000) >> 12
		if KvRoCo > 7:
			KvRoCo = KvRoCo - 16
		KvT[0] = KvRoCo
		KvReCo = (eeData[52] & 0x0F00) >> 8
		if KvReCo > 7:
			KvReCo = KvReCo - 16
		KvT[2] = KvReCo
		KvRoCe = (eeData[52] & 0x00F0) >> 4
		if KvRoCe > 7:
			KvRoCe = KvRoCe - 16
		KvT[1] = KvRoCe
		KvReCe = (eeData[52] & 0x000F)
		if KvReCe > 7:
			KvReCe = KvReCe - 16
		KvT[3] = KvReCe
		kvScale = (eeData[56] & 0x0F00) >> 8
		for i in range(24):
			for j in range(32):
				p = 32 * i +j
				split = 2*(p//32 - (p//64)*2) + p%2
				kvTemp[p] = KvT[split]
				kvTemp[p] = kvTemp[p] / (2**kvScale)		
		temp = abs(kvTemp[0])
		for i in range(1,768):
			if abs(kvTemp[i]) > temp:
				temp = abs(kvTemp[i])
		kvScale = 0
		while temp < 64:
			temp = temp*2
			kvScale = kvScale + 1
		self.kv = array.array( 'b', [0]*(COLS*ROWS) )
		for i in range(768):
			temp = kvTemp[i] * (2**kvScale)
			if temp < 0:
				self.kv[i] = int(temp - 0.5)
			else:
				self.kv[i] = int(temp + 0.5)
		self.kvScale = kvScale
		del KvT
		# ExtractCILCParameters
		ilChessC = [0]*3
		calibrationModeEE = (eeData[10] & 0x0800) >> 4
		calibrationModeEE = calibrationModeEE ^ 0x80
		ilChessC[0] = (eeData[53] & 0x003F)
		if ilChessC[0] > 31:
			ilChessC[0] = ilChessC[0] - 64
		ilChessC[0] = ilChessC[0] / 16.0
		ilChessC[1] = (eeData[53] & 0x07C0) >> 6
		if ilChessC[1] > 15:
			ilChessC[1] = ilChessC[1] - 32
		ilChessC[1] = ilChessC[1] / 2.0
		ilChessC[2] = (eeData[53] & 0xF800) >> 11
		if ilChessC[2] > 15:
			ilChessC[2] = ilChessC[2] - 32
		ilChessC[2] = ilChessC[2] / 8.0		
		self.calibrationModeEE = calibrationModeEE
		self.ilChessC = [ ilChessC[0], ilChessC[1], ilChessC[2] ]
		# ExtractDeviatingPixels  (NOT implemented.)
		self.brokenPixels = [0xFFFF]*5
		self.outlierPixels = [0xFFFF]*5

	def __str__(self):
		return 'kVdd:{},vdd25:{},\nKvPTAT:{},KtPTAT:{},vPTAT25:{},alphaPTAT:{},\ngainEE:{},tgc:{},cpKv:{},cpKta:{},\nresolutionEE:{},calibrationModeEE:{},\nKsTa:{},alphaScale:{},ktaScale:{},kvScale:{},\ncpAlpha[0]:{},cpAlpha[1]:{},\ncpOffset[0]:{},cpOffset[1]:{},\nilChessC[0]:{},ilChessC[1]:{},ilChessC[2]:{}'.format(self.kVdd,self.vdd25,self.KvPTAT,self.KtPTAT,self.vPTAT25,self.alphaPTAT,self.gainEE,self.tgc,self.cpKv,self.cpKta,self.resolutionEE,self.calibrationModeEE,self.KsTa,self.alphaScale,self.ktaScale,self.kvScale,self.cpAlpha[0],self.cpAlpha[1],self.cpOffset[0],self.cpOffset[1],self.ilChessC[0],self.ilChessC[1],self.ilChessC[2])

# sensor class
class MLX90640sensor:
	COLS = 32
	ROWS = 24
	ADR_RAM     = 0x0400
	ADR_E2PROM  = 0x2400
	REG_STATUS  = 0x8000
	REG_CTRL1   = 0x800D
	REG_CTRL2   = 0x800E
	REG_I2CCONF = 0x800F
	REG_I2CADDR = 0x8010
	def __init__( self, i2cid=I2C.I2C0, freq=100000, scl=34, sda=35, slave_addr=0x33 ):
		self.i2c_open(i2cid=i2cid,freq=freq,scl=scl,sda=sda,slave_addr=slave_addr)
	def getE2param( self ):
		eeData = self.i2c_read( self.ADR_E2PROM, 832 )
		self.param = MLX90640param(eeData,self.COLS,self.ROWS)
	def i2c_open( self, i2cid=I2C.I2C0, freq=100000, scl=34, sda=35, slave_addr=0x33 ):
		self.i2c = I2C( i2cid, freq=freq, scl=scl, sda=sda)
		self.i2c_devices = self.i2c.scan()
		if slave_addr not in self.i2c_devices:
			print('i2c: slave addr {} is not found.'.format(slave_addr))
			self.isopened = False
		else:
			self.slave_addr = slave_addr
			self.isopened = True
			self.getE2param()
			self.setRefreshRate(0x05)
			#self.setSubpageMode(1)
			self.pixels = None
		return self.isopened
	def i2c_close( self ):
		self.i2c.deinit()
	def magic( self ):	# magic code to stabilize i2c
		self.i2c.writeto( self.slave_addr, bytes([self.REG_STATUS>>8, self.REG_STATUS&0x00]) )
		self.i2c.readfrom( self.slave_addr, 2 )
	def i2c_read( self, addr, length, unit=512 ):
		self.magic()
		data = None
		startaddr = addr
		while length > 0:
			_length = unit if length >= unit else length
			_data = self.i2c.readfrom_mem( self.slave_addr, startaddr, _length*2, mem_size=16)
			if data is None:
				data = _data
			else:
				data = data + _data
				del _data
			startaddr += _length
			length    -= _length	
		return struct.unpack('>'+'H'*(len(data)//2), data)
	def i2c_write( self, addr, data ):
		self.magic()
		buf = struct.pack('>'+'H'*len(data), *data)
		self.i2c.writeto_mem( self.slave_addr, addr, buf, mem_size=16 )
	def setRefreshRate( self, refreshRate=0x05 ):
		'''
		// 0x00 – 0.5Hz
		// 0x01 – 1Hz
		// 0x02 – 2Hz
		// 0x03 – 4Hz
		// 0x04 – 8Hz // OK 
		// 0x05 – 16Hz // OK
		// 0x06 – 32Hz // Fail
		// 0x07 – 64Hz
		'''
		value = (refreshRate & 0x07)<<7
		controlRegister1 = self.i2c_read( self.REG_CTRL1, 1 )[0]
		value = (controlRegister1 & 0xFC7F) | value
		self.i2c_write( self.REG_CTRL1, (value,) )
	#def setSubpageMode( self, valid ):
	#	controlRegister1 = self.i2c_read( self.REG_CTRL1, 1 )[0]
	#	value = (controlRegister1 & (0xFFFE)) | valid
	#	self.i2c_write( self.REG_CTRL1, (value,) )
	def getCurMode( self ):
		controlRegister1 = self.i2c_read( self.REG_CTRL1, 1 )[0]
		modeRAM = (controlRegister1 & 0x1000) >> 12
		return modeRAM
	def captureFrameData( self, wantsubpage=-1, maxloop=5000 ):
		def enable_overwrite():
			statusRegister = self.i2c_read( self.REG_STATUS, 1 )[0]
			self.i2c_write( self.REG_STATUS, ((statusRegister | 0x0010),) )
		def disble_overwrite():
			statusRegister = self.i2c_read( self.REG_STATUS, 1 )[0]
			self.i2c_write( self.REG_STATUS, ((statusRegister & 0xFFEF),) )

		enable_overwrite()
		for loop in range(maxloop):
			statusRegister = self.i2c_read( self.REG_STATUS, 1 )[0]
			dataReady = statusRegister & 0x0008
			if dataReady != 0:
				subpage = statusRegister & 0x0001
				if wantsubpage < 0 or subpage == wantsubpage:
					break
		if loop >= (maxloop-1):
			return -8, None
		disble_overwrite()
		frameData = self.i2c_read( self.ADR_RAM, 832 )
		enable_overwrite()

		controlRegister1 = self.i2c_read( self.REG_CTRL1, 1 )[0]
		frameData = array.array('H',frameData)
		frameData.append( controlRegister1 )
		frameData.append( statusRegister & 0x0001 )
		#print('page: ', frameData[833])
		return frameData[833], frameData
	def preparePixels( self ):
		return array.array('f',[0]*768)
	def capturePixels( self, speed_setting=2 ):
		if self.pixels is None:
			self.pixels = self.preparePixels()
		return self._capturePixels( self.pixels, speed_setting=speed_setting )
	def _capturePixels( self, pixels=None, speed_setting=2, TA_SHIFT=8 ):
		resflag = 0
		if pixels is None:
			pixels = self.preparePixels()

		_frameData = []
		wantpage = -1 
		for x in range(speed_setting):
			subpage, frameData = self.captureFrameData(wantsubpage=wantpage)
			if subpage < 0:
				print('fail to captureFrameData', subpage)
				continue
			_frameData.append(frameData)
			wantpage = 1 - subpage
			resflag = resflag | (1<<subpage)
		
		for frameData in _frameData:
			vdd = self.getVdd( frameData )
			Ta = self.getTa( frameData )
			tr = Ta - TA_SHIFT
			self.calculateTo( pixels, frameData, tr )
			# BadPixelsCorrection not implemented.
			mode = self.getCurMode()
			if x < (speed_setting-1):
				del frameData
		return resflag, pixels
	def getVdd( self, frameData ):
		vdd = frameData[810]
		if vdd > 32767:
			vdd = vdd - 65536
		resolutionRAM = (frameData[832] & 0x0C00) >> 10
		resolutionCorrection = (2**self.param.resolutionEE) / (2**resolutionRAM)
		vdd = (resolutionCorrection * vdd - self.param.vdd25) / self.param.kVdd + 3.3
		return vdd
	def getTa( self, frameData ):
		vdd = self.getVdd(frameData)
		ptat = frameData[800]
		if ptat > 32767:
			ptat = ptat - 65536
		ptatArt = frameData[768]
		if ptatArt > 32767:
			ptatArt = ptatArt - 65536
		ptatArt = (ptat / (ptat * self.param.alphaPTAT + ptatArt)) * (2**18)		
		ta = (ptatArt / (1 + self.param.KvPTAT * (vdd - 3.3)) - self.param.vPTAT25)
		ta = ta / self.param.KtPTAT + 25
		return ta
	def calculateTo( self, pixels, frameData, tr, emissivity=0.95 ):
		irDataCP = [0]*2
		alphaCorrR = [0]*4
		subpage = frameData[833]
		vdd = self.getVdd(frameData)
		ta = self.getTa(frameData)
		ta4 = (ta + 273.15)
		ta4 = ta4 * ta4
		ta4 = ta4 * ta4
		tr4 = (tr + 273.15)
		tr4 = tr4 * tr4
		tr4 = tr4 * tr4
		taTr = tr4 - (tr4-ta4)/emissivity		
		ktaScale = 2**self.param.ktaScale
		kvScale = 2**self.param.kvScale
		alphaScale = 2**self.param.alphaScale	
		alphaCorrR[0] = 1 / (1 + self.param.ksTo[0] * 40)
		alphaCorrR[1] = 1
		alphaCorrR[2] = (1 + self.param.ksTo[1] * self.param.ct[2])
		alphaCorrR[3] = alphaCorrR[2] * (1 + self.param.ksTo[2] * (self.param.ct[3] - self.param.ct[2]))
		# Gain calculation
		gain = frameData[778]
		if gain > 32767:
			gain = gain - 65536
		gain = self.param.gainEE / gain
		# To calculation
		mode = (frameData[832] & 0x1000) >> 5		
		irDataCP[0] = frameData[776]
		irDataCP[1] = frameData[808]
		for i in range(2):
			if irDataCP[i] > 32767:
				irDataCP[i] = irDataCP[i] - 65536
			irDataCP[i] = irDataCP[i] * gain
		irDataCP[0] = irDataCP[0] - self.param.cpOffset[0] * (1 + self.param.cpKta * (ta - 25)) * (1 + self.param.cpKv * (vdd - 3.3))
		if mode == self.param.calibrationModeEE:
			irDataCP[1] = irDataCP[1] - self.param.cpOffset[1] * (1 + self.param.cpKta * (ta - 25)) * (1 + self.param.cpKv * (vdd - 3.3))
		else:
			irDataCP[1] = irDataCP[1] - (self.param.cpOffset[1] + self.param.ilChessC[0]) * (1 + self.param.cpKta * (ta - 25)) * (1 + self.param.cpKv * (vdd - 3.3))
		for pixelNumber in range(768):
			ilPattern = pixelNumber // 32 - (pixelNumber // 64) * 2
			chessPattern = ilPattern ^ (pixelNumber - (pixelNumber//2)*2)
			conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern)
			if mode == 0:
				pattern = ilPattern
			else:
				pattern = chessPattern
			if pattern == subpage:
				irData = frameData[pixelNumber]
				if irData > 32767:
					irData = irData - 65536
				irData = irData * gain
				kta = self.param.kta[pixelNumber]/ktaScale
				kv = self.param.kv[pixelNumber]/kvScale
				irData = irData - self.param.offset[pixelNumber]*(1 + kta*(ta - 25))*(1 + kv*(vdd - 3.3))
				if mode != self.param.calibrationModeEE:
					irData = irData + self.param.ilChessC[2] * (2 * ilPattern - 1) - self.param.ilChessC[1] * conversionPattern		
				irData = irData - self.param.tgc * irDataCP[subpage]
				irData = irData / emissivity
				alphaCompensated = self.param.SCALEALPHA*alphaScale/self.param.alpha[pixelNumber]
				alphaCompensated = alphaCompensated*(1 + self.param.KsTa * (ta - 25))
				Sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr)
				Sx = math.sqrt(math.sqrt(Sx)) * self.param.ksTo[1]
				To = math.sqrt(math.sqrt(irData/(alphaCompensated * (1 - self.param.ksTo[1] * 273.15) + Sx) + taTr)) - 273.15						
				if To < self.param.ct[1]:
					range = 0
				elif To < self.param.ct[2]:
					range = 1
				elif To < self.param.ct[3]:
					range = 2            
				else:
					range = 3
				To = math.sqrt(math.sqrt(irData / (alphaCompensated * alphaCorrR[range] * (1 + self.param.ksTo[range] * (To - self.param.ct[range]))) + taTr)) - 273.15
				pixels[pixelNumber] = To
	def snapshot( self, colortbl, pixels=None, scale=5, speed_setting=2, mintemp=21, maxtemp=38 ):
		if pixels is None:
			ret, pixels = self.capturePixels(speed_setting=speed_setting)
		img = image.Image(width=self.COLS*scale,height=self.ROWS*scale,format=sensor.RGB565)
		for y in range(self.ROWS):
			for x in range(self.COLS):
				val = pixels[ y*32+x ]
				val = (val - mintemp)/(maxtemp-mintemp)
				val = int(val*len(colortbl))
				if val < 0:
					val = 0
				if val >= len(colortbl):
					val = len(colortbl) - 1
				val = colortbl[val]
				for j in range(scale):
					for i in range(scale):
						img.set_pixel(x*scale+i,y*scale+j,val)
		return img
	@staticmethod
	def get_minmax_pixels(pixels):
		minval = 1000
		maxval = -1000
		for pix in pixels:
			if minval > pix:
				minval = pix
			if maxval < pix:
				maxval = pix
		return minval, maxval
	@staticmethod
	def create_color_table():	# RGB565
		coltbl = []
		for i in range(1<<5):	# B
			coltbl.append(i)
		for i in range(1<<6):	# B->GB
			coltbl.append((i<<5) | ((1<<5)-1))
		for i in range(1<<5):	# GB->G
			coltbl.append( (((1<<6)-1)<<5) | (((1<<5)-1) - i))
		for i in range(1<<5):	# G->RG
			coltbl.append( (((1<<6)-1)<<5) | (i<<11) )
		for i in range(1<<6):	# RG->R
			coltbl.append( (((1<<5)-1)<<11) | (((1<<6)-1-i)<<5) )
		for i in range(len(coltbl)):	# byte swap for img.set_pixel()
			coltbl[i] = (coltbl[i] >> 8) | ((coltbl[i]&0xFF)<<8)
		return array.array('H',coltbl)
	@staticmethod
	def draw_colorbar(lcd, coltbl, xpos, step=10, swapbytes=False):	# set swapbytes as True if the bar looks reversed.
		height = lcd.height()
		vec = len(coltbl)/height
		msg = ' '
		for y in range(0,height,step):
			val = coltbl[ int(y*vec) ]
			if swapbytes:
				val = (val >> 8) | ((val&0xFF)<<8)
			lcd.draw_string(xpos,y, msg, val, val)
			del val
		del msg

def _main():
	import gc
	gc.collect()
	import lcd
	lcd.init(freq=15000000)

	# open
	mlx = MLX90640sensor(i2cid=I2C.I2C0, freq=100000, scl=34, sda=35, slave_addr=0x33)
	print('I2C devices :', mlx.i2c_devices)
	if not mlx.isopened:
		return -1
	#print(mlx.param)
	coltbl = mlx.create_color_table()
	mintemp = 21
	maxtemp = 38

	while True:
		#ret, pixels = mlx.captureFrameData()	# raw data
		ret, pixels = mlx.capturePixels( speed_setting=2 )	# temp data,  speed_setting= 2: 2pages and slow, 1: 1page and fast.

		# decide temp range
		minpix, maxpix = mlx.get_minmax_pixels( pixels )
		if minpix > mintemp:
			minpix = mintemp
		if maxpix < maxtemp:
			maxpix = maxtemp
		
		# create image and show
		#  scale=5, because sensor size 32x24, M5StickV LCD 240x135
		img = mlx.snapshot( coltbl, pixels=pixels, scale=5, mintemp=minpix, maxtemp=maxpix)

		lcd.display(img, (0,0,img.width(),img.height()))
		lcd.draw_string(200,   0, '{:.1f}'.format(minpix), lcd.WHITE, lcd.NAVY)
		lcd.draw_string(200,  55, '{:.1f}'.format((minpix+maxpix)/2), lcd.WHITE, lcd.DARKGREEN)
		lcd.draw_string(200, 110, '{:.1f}'.format(maxpix), lcd.WHITE, lcd.MAROON)
		mlx.draw_colorbar(lcd, coltbl, xpos=180 )	# causes out of memory. should not be called.
		del pixels
		del img
	mlx.i2c_close()
	return 0

_main()
