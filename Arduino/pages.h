const char page1[] PROGMEM = // todo: use gzip
   "<!DOCTYPE html>\n"
   "<html lang=\"en\">\n"
   "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"/>\n"
   "<title>WiFi UT181A Web Page</title>\n"
   "<style type=\"text/css\">\n"
   "input{\n"
   "border-radius: 5px;\n"
   "margin-bottom: 5px;\n"
   "box-shadow: 2px 2px 12px #000000;\n"
   "background-image: -moz-linear-gradient(top, #ffffff, #50a0ff);\n"
   "background-image: -ms-linear-gradient(top, #ffffff, #50a0ff);\n"
   "background-image: -o-linear-gradient(top, #ffffff, #50a0ff);\n"
   "background-image: -webkit-linear-gradient(top, #efffff, #50a0ff);\n"
   "background-image: linear-gradient(top, #ffffff, #50a0ff);\n"
   "background-clip: padding-box;\n"
   "}\n"
   "body{width:340px;font-family: Arial, Helvetica, sans-serif;}\n"
   ".range{\n"
   "  overflow: hidden;\n"
   "}\n"
   ".dropdown {\n"
   "    position: relative;\n"
   "    display: inline-block;\n"
   "}\n"
   ".dropbtn {\n"
   "    background-color: #50a0ff;\n"
   "    padding: 1px;\n"
   "    font-size: 12px;\n"
   "background-image: -webkit-linear-gradient(top, #efffff, #50a0ff);\n"
   "border-radius: 5px;\n"
   "margin-bottom: 5px;\n"
   "box-shadow: 2px 2px 12px #000000;\n"
   "}\n"
   ".btn {\n"
   "    background-color: #50a0ff;\n"
   "    padding: 1px;\n"
   "    font-size: 12px;\n"
   "    min-width: 56px;\n"
   "    border: none;\n"
   "}\n"
   ".dropdown-content {\n"
   "    display: none;\n"
   "    position: absolute;\n"
   "    background-color: #919191;\n"
   "    min-width: 60px;\n"
   "    min-height: 120px;\n"
   "    z-index: 1;\n"
   "}\n"
   ".dropdown:hover .dropdown-content {display: block;}\n"
   ".dropdown:hover .dropbtn {background-color: #3e8e41;}\n"
   ".dropdown:hover .btn {background-image: -webkit-linear-gradient(top, #efffff, #50a0ff);}\n"
   "</style>\n"
   "<script type=\"text/javascript\">\n"
   "a=document.all\n"
   "oledon=0;v=0;min=-60;max=60;mod=5;st=1073676544\n"
   "va=new Array()\n"
   "function startWS(){\n"
   "draw_bar()\n"
   "draw_chart()\n"
   "a.mm.innerHTML=\"MAX MIN\";\n"
   "ws = new WebSocket(\"ws://192.168.0.109/ws\")\n"
   "ws.onopen = function(evt) { }\n"
   "ws.onclose = function(evt) { alert(\"Connection closed.\"); }\n"
   "ws.onmessage = function(evt) {\n"
   "// console.log(evt.data)\n"
   " lines = evt.data.split(';')\n"
   " event=lines[0]\n"
   " data=lines[1]\n"
   " if(event == 'settings')\n"
   " {\n"
   " d=JSON.parse(data)\n"
   " oledon=d.o\n"
   " a.OLED.value=oledon?'ON ':'OFF'\n"
   " }\n"
   " if(event == 'state')\n"
   " {\n"
   " d=JSON.parse(data)\n"
   " dt=new Date(d.t*1000)\n"
   " a.time.innerHTML=dt.toLocaleTimeString()\n"
   " a.value.innerHTML=d.v\n"
   " a.unit.innerHTML=d.u.replace(/~/g, '&#937;')\n"
   " a.v1.innerHTML=d.v1\n"
   " a.v2.innerHTML=d.v2\n"
   " a.v3.innerHTML=d.v3\n"
   " v=+d.v\n"
   " va.push(v)\n"
   " min=+d.mn\n"
   " max=+d.mx\n"
   " mod=+d.md\n"
   " st=+d.st\n"
   " draw_bar()\n"
   " draw_chart()\n"
   " a.am.innerHTML=(st&(1<<4))?\"AUTO\":\"MANUAL\";\n"
   " a.h.innerHTML=(st&(1<<5))?\"HOLD\":\"\";\n"
   " a.mm.innerHTML=(st&(1<<11))?\"MAX MIN\":((st&1<<10)?\"REL\":\"\");\n"
   " }\n"
   " else if(event == 'alert')\n"
   " {\n"
   "  alert(data)\n"
   " }\n"
   " else if(event == 'range')\n"
   " {\n"
   "  d=JSON.parse(data)\n"
   "  for(i=0;i<8;i++){\n"
   "   item=document.getElementById('r'+i)\n"
   "   item.setAttribute('style',i<d.r.length?'':'display:none')\n"
   "//   if(i==idx) item.setAttribute('style','background-color:red')\n"
   "   item.innerHTML=d.r[i]\n"
   "  }\n"
   "  for(i=0;i<5;i++){\n"
   "   item=document.getElementById('o'+i)\n"
   "   item.setAttribute('style',i<d.o.length?'':'display:none')\n"
   "//   if(i==idx) item.setAttribute('style','background-color:red')\n"
   "   item.innerHTML=d.o[i]\n"
   "  }\n"
   " }\n"
   "}\n"
   "}\n"
   "function setVar(varName, value)\n"
   "{\n"
   " ws.send('cmd;{\"'+varName+'\":'+value+'}')\n"
   "}\n"
   "function setA(n)\n"
   "{\n"
   " setVar('range', n)\n"
   "}\n"
   "function setOpt(n)\n"
   "{\n"
   " setVar('sel', n)\n"
   "}\n"
   "function oled(){\n"
   "oledon=!oledon\n"
   "setVar('oled', oledon)\n"
   "a.OLED.value=oledon?'ON ':'OFF'\n"
   "}\n"
   "\n"
   "function draw_bar(){\n"
   "try {\n"
   "var c2 = document.getElementById('bar')\n"
   "ctx = c2.getContext(\"2d\")\n"
   "ctx.fillStyle = \"#000\";\n"
   "ctx.fillRect(0,0,c2.width,c2.height)\n"
   "ctx.fillStyle = \"#FFF\";\n"
   "ctx.lineWidth = 1\n"
   "ctx.strokeStyle = \"#FFF\"\n"
   "ctx.font = \"bold 10px sans-serif\";\n"
   "\n"
   "w=c2.width-30\n"
   "  \n"
   "for(i=0;i<=60;i++)\n"
   "{\n"
   "fVal=(i*(max-min)/60)+min\n"
   "x=i/60*w+12\n"
   "if((i%mod)==0)\n"
   "{\n"
   "ctx.fillText(fVal.toFixed(), x-(String(fVal.toFixed()).length/2)*5, 10)\n"
   "ctx.beginPath()\n"
   "    ctx.moveTo(x, 15);\n"
   "    ctx.lineTo(x, 24);\n"
   "}\n"
   "else\n"
   "{\n"
   "ctx.beginPath()\n"
   "    ctx.moveTo(x, 18);\n"
   "    ctx.lineTo(x, 24);\n"
   "}\n"
   "ctx.stroke()\n"
   "}\n"
   "\n"
   "if(min<0)\n"
   "{\n"
   "c=w>>1\n"
   "x=v/max*c\n"
   "ctx.fillStyle = \"#FFF\";\n"
   "if(x<0)\n"
   "{\n"
   "if(x<-c) x=-c\n"
   "ctx.fillRect(12+c+x, 30, -x, 6);\n"
   "}\n"
   "else\n"
   "{\n"
   "if(x>c) x=c\n"
   "ctx.fillRect(12+c, 30, x, 6);\n"
   "}\n"
   "ctx.fillStyle = \"#F00\";\n"
   "ctx.fillRect(12+c+x, 30, 1, 6);\n"
   "}\n"
   "else // unsigned\n"
   "{\n"
   "x = v/max*w\n"
   "if(x>w) x=w\n"
   "ctx.fillStyle = \"#FFF\";\n"
   "ctx.fillRect(12, 30, x, 6);\n"
   "ctx.fillStyle = \"#F00\";\n"
   "ctx.fillRect(12+x, 30, 1, 6);\n"
   "}\n"
   "}catch(err){}\n"
   "}\n"
   "function draw_chart(){\n"
   "try {\n"
   "var c=document.getElementById('chart')\n"
   "ctx = c.getContext(\"2d\")\n"
   "if(va.length>c.width-19) va.shift()\n"
   "ctx.fillStyle = \"#222\"\n"
   "ctx.fillRect(0,0,c.width,c.height)\n"
   "ctx.fillStyle = \"#FFF\"\n"
   "ctx.lineWidth = 1\n"
   "ctx.font = \"bold 10px sans-serif\"\n"
   "ctx.textAlign=\"right\"\n"
   "h=c.height-20\n"
   "fVal = min\n"
   "for(i=6;i>=0;i--)\n"
   "{\n"
   "y=i/6*h+12\n"
   "ctx.strokeStyle = \"#FFF\"\n"
   "ctx.fillText(fVal, 20, y+3)\n"
   "fVal+=(max-min)/6\n"
   "ctx.strokeStyle = \"#555\"\n"
   "ctx.beginPath()\n"
   "    ctx.moveTo(21, y)\n"
   "    ctx.lineTo(c.width, y)\n"
   "ctx.stroke()\n"
   "}\n"
   "if(min<0){\n"
   "base=h/2\n"
   "range=h/2\n"
   "}else{\n"
   "base=h\n"
   "range=h\n"
   "}\n"
   "ciel=max\n"
   "y=base-(va[0]/ciel*range)\n"
   "ctx.strokeStyle = \"#FFF\"\n"
   "ctx.beginPath()\n"
   "    ctx.moveTo(20, y);\n"
   "for(i=1,x=21;i<va.length;i++,x++)\n"
   "{\n"
   "y=base-(va[i]/ciel*range)\n"
   "    ctx.lineTo(x, y)\n"
   "}\n"
   "ctx.stroke()\n"
   "}catch(err){}\n"
   "}\n"
   "</script>\n"
   "<body bgcolor=\"black\" text=\"silver\"  onload=\"{startWS()}\">\n"
   "<div><h3 align=\"center\">WiFi UT181A</h3>\n"
   "\n"
   "<table align=center width=320>\n"
   "<tr align=center><td colspan=2><div id=\"time\" style=\"font-size:small\"> 0:00:00 AM</div></td><td width=70></td><td></td></tr>\n"
   "<tr align=center><td width=50><td><div id=\"mm\" width=70></div></td><td width=50><div id=\"h\"></div></td><td width=70><div id=\"am\">AUTO</div></td></tr>\n"
   "</table>\n"
   "<table align=center width=320>\n"
   "<tr align=center><td colspan=2><div id=\"value\" style=\"font-size:xx-large\">0L</div></td><td colspan=2><div id=\"unit\">DCV</div></td></tr>\n"
   "<tr><td colspan=4><canvas id=\"bar\" width=\"300\" height=\"40\" style=\"float:center\"></td></tr>\n"
   "<tr><td><div id=\"l1\"></div></td><td><div id=\"v1\">0L</div></td><td><div id=\"u1\"></div></td><td></td></tr>\n"
   "<tr><td><div id=\"l2\"></div></td><td><div id=\"v2\">0L</div></td><td><div id=\"u2\"></div></td><td></td></tr>\n"
   "<tr><td><div id=\"l3\"></div></td><td><div id=\"v3\">0L</div></td><td><div id=\"u3\"></div></td><td></td></tr>\n"
   "<tr><td colspan=4><input type=\"button\" value=\"Hold\" id=\"hold\" onClick=\"{setVar('hold', 0)}\">\n"
   "<div class=\"dropdown\">\n"
   "  <button class=\"dropbtn\">Range</button>\n"
   "  <div class=\"dropdown-content\">\n"
   "  <button class=\"btn\" id=\"r0\" onclick=\"setA(0)\">Auto</button>\n"
   "  <button class=\"btn\" id=\"r1\" onclick=\"setA(1)\">6</button>\n"
   "  <button class=\"btn\" id=\"r2\" onclick=\"setA(2)\">60</button>\n"
   "  <button class=\"btn\" id=\"r3\" onclick=\"setA(3)\">600</button>\n"
   "  <button class=\"btn\" id=\"r4\" onclick=\"setA(4)\">6000</button>\n"
   "  <button class=\"btn\" id=\"r5\" onclick=\"setA(5)\">10000</button>\n"
   "  <button class=\"btn\" id=\"r6\" onclick=\"setA(6)\"></button>\n"
   "  <button class=\"btn\" id=\"r7\" onclick=\"setA(7)\"></button>\n"
   "  <button class=\"btn\" id=\"r8\" onclick=\"setA(8)\"></button>\n"
   "  </div>\n"
   "</div>\n"
   "<div class=\"dropdown\">\n"
   "  <button class=\"dropbtn\">Option</button>\n"
   "  <div class=\"dropdown-content\">\n"
   "  <button class=\"btn\" id=\"o0\" onclick=\"setOpt(0)\">Opt</button>\n"
   "  <button class=\"btn\" id=\"o1\" onclick=\"setOpt(1)\">0</button>\n"
   "  <button class=\"btn\" id=\"o2\" onclick=\"setOpt(2)\">0</button>\n"
   "  <button class=\"btn\" id=\"o3\" onclick=\"setOpt(3)\">0</button>\n"
   "  <button class=\"btn\" id=\"o4\" onclick=\"setOpt(4)\">0</button>\n"
   "  <button class=\"btn\" id=\"o5\" onclick=\"setOpt(5)\"></button>\n"
   "  </div>\n"
   "</div>\n"
   "<input type=\"button\" value=\"REL\" id=\"REL\" onClick=\"{setVar('rel', 0)}\">\n"
   "<input id='rval' type=text size=4 value='0'> &nbsp <input type=\"button\" value=\"M/M\" id=\"M/M\" onClick=\"{setVar('mm', 0)}\"></td></tr>\n"
   "<tr><td colspan=4></td></tr>\n"
   "<tr><td colspan=4><canvas id=\"chart\" width=\"300\" height=\"300\" style=\"float:center\"></td></tr>\n"
   "<tr><td colspan=4>&nbsp </td></tr>\n"
   "<tr><td>OLED: <input type=\"button\" value=\"ON\" id=\"OLED\" onClick=\"{oled()}\"></td><td></td><td><input type=\"submit\" value=\"Setup\" onClick=\"window.location='/s';\"></td><td></td></tr>\n"
   "</table>\n"
   "&nbsp\n"
   "</div>\n"
   "</body>\n";

const uint8_t favicon[] PROGMEM = {
  0x1F, 0x8B, 0x08, 0x08, 0x70, 0xC9, 0xE2, 0x59, 0x04, 0x00, 0x66, 0x61, 0x76, 0x69, 0x63, 0x6F, 
  0x6E, 0x2E, 0x69, 0x63, 0x6F, 0x00, 0xD5, 0x94, 0x31, 0x4B, 0xC3, 0x50, 0x14, 0x85, 0x4F, 0x6B, 
  0xC0, 0x52, 0x0A, 0x86, 0x22, 0x9D, 0xA4, 0x74, 0xC8, 0xE0, 0x28, 0x46, 0xC4, 0x41, 0xB0, 0x53, 
  0x7F, 0x87, 0x64, 0x72, 0x14, 0x71, 0xD7, 0xB5, 0x38, 0x38, 0xF9, 0x03, 0xFC, 0x05, 0x1D, 0xB3, 
  0x0A, 0x9D, 0x9D, 0xA4, 0x74, 0x15, 0x44, 0xC4, 0x4D, 0x07, 0x07, 0x89, 0xFA, 0x3C, 0x97, 0x9C, 
  0xE8, 0x1B, 0xDA, 0x92, 0x16, 0x3A, 0xF4, 0x86, 0x8F, 0x77, 0x73, 0xEF, 0x39, 0xEF, 0xBD, 0xBC, 
  0x90, 0x00, 0x15, 0x5E, 0x61, 0x68, 0x63, 0x07, 0x27, 0x01, 0xD0, 0x02, 0xB0, 0x4D, 0x58, 0x62, 
  0x25, 0xAF, 0x5B, 0x74, 0x03, 0xAC, 0x54, 0xC4, 0x71, 0xDC, 0x35, 0xB0, 0x40, 0xD0, 0xD7, 0x24, 
  0x99, 0x68, 0x62, 0xFE, 0xA8, 0xD2, 0x77, 0x6B, 0x58, 0x8E, 0x92, 0x41, 0xFD, 0x21, 0x79, 0x22, 
  0x89, 0x7C, 0x55, 0xCB, 0xC9, 0xB3, 0xF5, 0x4A, 0xF8, 0xF7, 0xC9, 0x27, 0x71, 0xE4, 0x55, 0x38, 
  0xD5, 0x0E, 0x66, 0xF8, 0x22, 0x72, 0x43, 0xDA, 0x64, 0x8F, 0xA4, 0xE4, 0x43, 0xA4, 0xAA, 0xB5, 
  0xA5, 0x89, 0x26, 0xF8, 0x13, 0x6F, 0xCD, 0x63, 0x96, 0x6A, 0x5E, 0xBB, 0x66, 0x35, 0x6F, 0x2F, 
  0x89, 0xE7, 0xAB, 0x93, 0x1E, 0xD3, 0x80, 0x63, 0x9F, 0x7C, 0x9B, 0x46, 0xEB, 0xDE, 0x1B, 0xCA, 
  0x9D, 0x7A, 0x7D, 0x69, 0x7B, 0xF2, 0x9E, 0xAB, 0x37, 0x20, 0x21, 0xD9, 0xB5, 0x33, 0x2F, 0xD6, 
  0x2A, 0xF6, 0xA4, 0xDA, 0x8E, 0x34, 0x03, 0xAB, 0xCB, 0xBB, 0x45, 0x46, 0xBA, 0x7F, 0x21, 0xA7, 
  0x64, 0x53, 0x7B, 0x6B, 0x18, 0xCA, 0x5B, 0xE4, 0xCC, 0x9B, 0xF7, 0xC1, 0xBC, 0x85, 0x4E, 0xE7, 
  0x92, 0x15, 0xFB, 0xD4, 0x9C, 0xA9, 0x18, 0x79, 0xCF, 0x95, 0x49, 0xDB, 0x98, 0xF2, 0x0E, 0xAE, 
  0xC8, 0xF8, 0x4F, 0xFF, 0x3F, 0xDF, 0x58, 0xBD, 0x08, 0x25, 0x42, 0x67, 0xD3, 0x11, 0x75, 0x2C, 
  0x29, 0x9C, 0xCB, 0xF9, 0xB9, 0x00, 0xBE, 0x8E, 0xF2, 0xF1, 0xFD, 0x1A, 0x78, 0xDB, 0x00, 0xEE, 
  0xD6, 0x80, 0xE1, 0x90, 0xFF, 0x90, 0x40, 0x1F, 0x04, 0xBF, 0xC4, 0xCB, 0x0A, 0xF0, 0xB8, 0x6E, 
  0xDA, 0xDC, 0xF7, 0x0B, 0xE9, 0xA4, 0xB1, 0xC3, 0x7E, 0x04, 0x00, 0x00, 
};
