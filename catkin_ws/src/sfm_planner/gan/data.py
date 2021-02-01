import torch


class Dataset(torch.utils.data.Dataset):

    def __init__(self, x):
        self.data = x

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return self.data[idx]
