
import torch
import torch.nn as nn
import torchvision

class Identity(nn.Module):
    """
    Author: Janne Spijkervet
    url: https://github.com/Spijkervet/SimCLR
    """

    def __init__(self):
        super(Identity, self).__init__()

    def forward(self, x):
        return x

class GripperNet(nn.Module):
    def __init__(self, dropout: float = 0.5):
        super(GripperNet, self).__init__()

        self.resnet18 = torchvision.models.resnet18(
            weights=torchvision.models.ResNet18_Weights.DEFAULT
        )
        self.resnet18.fc = Identity()
        self.avgpool = nn.AdaptiveAvgPool2d((6, 6))
        self.regressor = nn.Sequential(
            nn.Dropout(p=dropout),
            nn.Linear(512, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(p=dropout),
            nn.Linear(256, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, 1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.resnet18(x)
        x = torch.sigmoid(self.regressor(x))
        return x

