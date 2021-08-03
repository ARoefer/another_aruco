import cv2
import os

import subprocess

from argparse    import ArgumentParser
from collections import namedtuple
from jinja2      import Template

RowItem = namedtuple('RowItem', ['img', 'id'])


if __name__ == '__main__':
	dicts = {x[5:]: getattr(cv2.aruco, x) for x in dir(cv2.aruco) if x[:5] == 'DICT_'}

	parser = ArgumentParser(description='Tool to create a Pdf file containing desired'
										'aruco markers. This tool assumes that you have latex installed.')
	parser.add_argument('filename', help='Name of the created Pdf file')
	parser.add_argument('dict', help='Aruco dictionary to use. Options: {}'.format(', '.join(dicts.keys())))
	parser.add_argument('marker_width', type=float, help='Width of the markers in centimeters')
	parser.add_argument('marker_ids', nargs='+', help='List of markers to generate. '
													  'Can also be specified as ranges, e.g. 43-56')

	args = parser.parse_args()

	args.filename = args.filename[:-4] if args.filename[-4:].lower() == '.pdf' else args.filename

	if args.dict.upper() not in dicts:
		print(f'Unknown dictionary {args.dict}')
		exit()

	args.dict = args.dict.upper()

	ids = set()
	for x in args.marker_ids:
		if x.count('-') > 1 or x[0] == '-' or x[-1] == '-':
			print(f'Invalid marker id argument {x}')
			exit()

		if '-' not in x:
			ids.add(int(x))
		else:
			a, b = x.split('-')
			a = int(a)
			b = int(b)
			ids.update(range(min(a, b), max(a, b) + 1))

	ids = sorted(list(ids))
	marker_dict = cv2.aruco.Dictionary_get(dicts[args.dict])
	if ids[-1] > marker_dict.bytesList.shape[0]:
		print(f'Marker ids are out of range. Max id for this dict is {marker_dict.bytesList.shape[0] - 1}')

	dir_of_this_script = '/'.join(f'{os.getcwd()}/{__file__}'.split('/')[:-1])
	with open(f'{dir_of_this_script}/../res/marker_pdf_template.tex', 'r') as f:
		tex_template = Template(f.read())

	# usable width in cm
	paper_width = 19.0 

	items = []

	for i in ids:
		img = marker_dict.drawMarker(i, marker_dict.markerSize + 2, borderBits=1)
		img_path = f'{args.dict}_{i}.png'
		cv2.imwrite(img_path, img)
		items.append(RowItem(img_path, i))

	bit_width = args.marker_width / (marker_dict.markerSize + 2)
	items_per_row = int(paper_width // (args.marker_width + 2*bit_width))

	rows = []
	while len(items) > 0:
		rows.append(items[:items_per_row])
		items = items[items_per_row:]

	with open(f'{args.filename}.tex', 'w') as f:
		f.write(tex_template.render(box_border=bit_width, 
								    rows=rows, 
								    img_width=args.marker_width,
								    box_width=args.marker_width+2*bit_width,
								    img_offset=0.5*args.marker_width+bit_width))

	print(f'Generated {args.filename}.tex. Running tex...')

	subprocess.run(['pdflatex', f'{args.filename}.tex'])

	print('Done generating pdf. Removing temporary files...')
	for item in sum(rows, []):
		os.remove(item.img)

	os.remove(f'{args.filename}.tex')
	os.remove(f'{args.filename}.aux')
	os.remove(f'{args.filename}.log')

	print('Enjoy your markers!')
