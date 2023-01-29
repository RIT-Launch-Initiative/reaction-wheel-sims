function table = openrocket_post(filepath, start_event, end_event)
	% Read OpenRocket output to table object
	% filepath:		path to csv file
	% start_event: 	name of the event to start from (e.g. LAUNCH, EJECTION_CHARGE)
	% end_event: 	name of the event to end on (e.g. APOGEE)
	
	header_line_no = 1;
	event = "# Event %s";
	start_tag = sprintf(event, start_event);
	end_tag = sprintf(event, end_event);

	file_lines = readlines(filepath, "Encoding", "UTF-8");
	% grab headers
	headers_char = char(file_lines(header_line_no));
	stripped = regexprep(headers_char(3:end), "\s?\(.*?\)\s?", "");

	% grab data between tags
	start_index = find(startsWith(file_lines, start_tag));
	end_index = find(startsWith(file_lines, end_tag));
	interval_lines = file_lines(start_index + 1:end_index - 1);
	% remove any intervening tags
	data = interval_lines(~startsWith(interval_lines, '#'));
	new_lines = cat(1, stripped, data); 

	% the methods that auto-magically parse the data work on files, not strings
	% so after pre-processing the output (stripping out things before start and after end),
	% we write the data to a temp file and immediately read it back

	% write and read from temporary file
	tn = sprintf("%s.csv", tempname);
	fid = fopen(tn, "a+");
	writelines(new_lines, tn, WriteMode = "overwrite");
	fclose(fid);

	% read table from temporary file
	table = readtable(tn);

	% save table to file 
	[~, filename, ~] = fileparts(filepath);
	save(sprintf('%s.mat', filename), 'table');
end
