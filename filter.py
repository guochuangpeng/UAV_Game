class FirstOrderFilter:

    def __init__(self, filter_alpha) -> None:
        self.filter_val = 0
        self.filter_alpha = filter_alpha

    def get_value(self, now_value):
        out = self.filter_alpha * now_value + (1-self.filter_alpha) * self.filter_val
        self.filter_val = out

        return out
    
    def reset(self):
        self.filter_val = 0


