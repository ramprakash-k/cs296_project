import re
f=open("./../doc/cs296_project_report.tex",'r')
g=open("./../doc/cs296_project_report.html",'w')
tex=f.readlines()
g.write("""<html>
<head>
<title>CS296 Project Report</title>
<style>
h2{margin-left: 5%;}
</style>
</head>
<body>
<h1 style="margin-bottom:10;"><center> Group 1 Project Report </center></h1>
<table align=center frame=none cellspacing=5 width=1300>
""")
start1=-1
end1=-1
cnt=0
for i in range(len(tex)):
	if tex[i].find('author')!=-1 and start1==-1:
		start1=i
		cnt=cnt+tex[i].count('{')-tex[i].count('}')
	elif start1!=-1:
		if cnt>0:
			cnt=cnt+tex[i].count('{')-tex[i].count('}')
		else:
			end1=i+1
			break
	
auth=[[],[],[]]
for i in range(start1,end1):
	if re.search(r'([^ ]*) (.*)\\\\(.*)',tex[i]):
		auth[0].append(re.sub(r'([^ ]*) (.*)\\\\(.*)',r'<td><center> \2 </center></td>',tex[i]))
	if re.search('([0-9]+)\\\\(.*)',tex[i]):
		auth[1].append(re.sub('([0-9]+)\\\\(.*)',r'<td><center> \1 </center></td>',tex[i]))
	if re.search('\\\\texttt{(.*)}(.*)',tex[i]):
		auth[2].append(re.sub('\\\\texttt{(.*)}(.*)',r'<td><center> \1 </center></td>',tex[i]))

for i in range(3):
	g.write("<tr>\n")
	for j in range(len(auth[i])):
		g.write(auth[i][j])
	g.write("</tr>\n")

g.write("""</table>
<ul style="list-style:none"><font size="5">
""")

sup = re.compile('\\\textsuperscript{(.*)}')
intro = 0
figure = 1
start = -1
end = len(tex)
for i in range(len(tex)):
	if start==-1 and re.search('\\\section{([a-z A-Z]*)}',tex[i]):
		start=i
		break

for i in range(start,end):
	if(tex[i].strip()==''):
		continue
	else:
		less = re.sub('\\\\textless',r'<',tex[i])
		grt = re.sub('\\\\textgreater',r'>',less)
		cite= re.sub('\\\\cite{[a-z]*}','',grt)
		ital = re.sub('\\\\textit{([a-zA-Z ()/0-9\\\_:.<>\-\+]*)}',r'<i>\1</i>',cite)
		bk = re.sub('\\\_',r'_',ital)
		sec = re.sub('\\\section{(.*)}',r'<li><h2>\1</h2></li>',bk)
		subsec = re.sub('\\\subsection{(.*)}',r'<li><b>\1</b></li>',sec)
		image = re.sub('\\\includegraphics\[.*]{(.*)}',r'<center><img src="./\1"></center>',subsec)
		sup = re.sub('\\\\textsuperscript{(.*)}',r'<sup>\1</sup>',image)
		br = re.sub(r'\\\\',r'<br>',sup)
		cap = re.sub('\\\caption{(.*)}',r'<center><p><font size="3">Figure ' + str(figure) + r': \1</font></p></center>',br)
		if re.search('\\\caption{(.*)}',br):
			figure = figure + 1
		if re.search('\\\section{(.*)}',ital) and intro>1:
			g.write('</ol>\n')
			if cap[0]!='\\':
				g.write(cap)
			g.write('<ol>\n')
		elif re.search('\\\section{(.*)}',ital) and intro==1:
			if cap[0]!='\\':
				g.write(cap)
			g.write('<ol>\n')
		else:
			if cap[0]!='\\':
				g.write(cap)
		if re.search('\\\section{(.*)}',ital):
			intro=intro+1

g.write("""</ol>
</font></ul>
</body>
</html>""")
