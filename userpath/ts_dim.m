function ts = ts_dim(ts, varargin)
	% Extract sel from ts
	ts = timeseries(ts.Data(:, varargin{:}), ts.Time);
end
