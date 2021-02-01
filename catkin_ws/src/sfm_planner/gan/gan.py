import time

import torch

import utils


class GAN:
    def __init__(self, generator, discriminator, z_dim, lr=0.0001, wasserstein=False, device="cpu"):
        self.G = generator
        self.D = discriminator
        self.dv = device
        self.wasserstein = wasserstein
        self.z_dim = z_dim
        self.optim_g = torch.optim.Adam(lr=lr, params=self.G.parameters(), betas=(0.5, 0.999), amsgrad=True)
        self.optim_d = torch.optim.Adam(lr=lr, params=self.D.parameters(), betas=(0.5, 0.999), amsgrad=True)
        if not wasserstein:
            self.criterion = torch.nn.BCEWithLogitsLoss()

    def train(self, loader, epoch, c_iter, verbose=True):
        N = len(loader.dataset.data)
        bs = loader.batch_size
        loop_per_epoch = N // (bs * c_iter)

        for e in range(epoch):
            gen_avg_loss = 0.0
            disc_avg_loss = 0.0
            iterator = iter(loader)
            start = time.time()
            for i in range(loop_per_epoch):
                # TRAIN DISCRIMINATOR
                # turn off grad computation for the generator
                utils.switch_grads(self.G, False)
                utils.switch_grads(self.D, True)
                for c in range(c_iter):
                    # discriminator loss for real data
                    self.optim_d.zero_grad()
                    x_real = iterator.next()
                    x_real = x_real.to(self.dv)
                    logits_real = self.D(x_real)

                    # discriminator loss for fake data
                    z = torch.randn(bs, self.z_dim, device=self.dv)
                    x_fake = self.G(z)
                    logits_fake = self.D(x_fake)

                    # compute loss & gradients
                    if self.wasserstein:
                        loss = - logits_real.mean() + logits_fake.mean()
                        # lipschitz condition
                        loss += utils.gradient_penalty(self.D, x_real, x_fake, 1.0, self.dv)
                    else:
                        real_loss = self.criterion(logits_real, torch.ones_like(logits_real, device=self.dv))
                        fake_loss = self.criterion(logits_fake, torch.zeros_like(logits_fake, device=self.dv))
                        loss = real_loss + fake_loss

                    loss.backward()
                    self.optim_d.step()
                    disc_avg_loss += loss.item()

                # TRAIN GENERATOR
                # turn off grad computation for the discriminator
                utils.switch_grads(self.G, True)
                utils.switch_grads(self.D, False)

                self.optim_g.zero_grad()
                z = torch.randn(bs, self.z_dim, device=self.dv)
                x_fake = self.G(z)
                logits_fake = self.D(x_fake)

                if self.wasserstein:
                    loss = - logits_fake.mean()
                else:
                    loss = self.criterion(logits_fake, torch.ones_like(logits_fake, device=self.dv))

                loss.backward()
                self.optim_g.step()
                gen_avg_loss += loss.item()

            end = time.time()
            time_elapsed = end - start
            gen_avg_loss /= loop_per_epoch
            disc_avg_loss /= (loop_per_epoch * c_iter)
            if verbose:
                if e+1 == 1:
                    eta = time_elapsed * epoch
                    finish = time.asctime(time.localtime(time.time()+eta))
                    print("### Set your alarm at:", finish, "###")
                print("Epoch: %d - D loss: %.5f - G loss: %.5f - Time Elapsed: %.2f" % (e+1, disc_avg_loss, gen_avg_loss, time_elapsed))

        utils.switch_grads(self.G, True)
        utils.switch_grads(self.D, True)

    def generate(self, n=1):
        self.G.eval()
        z = torch.randn(n, self.z_dim, device=self.dv)
        with torch.no_grad():
            x = self.G(z).cpu()
        self.G.train()
        return x

    def discriminate(self, x):
        self.D.eval()
        with torch.no_grad():
            logits = self.D(x)
        self.D.train()
        return logits

    def save(self, name):
        torch.save(self.D.eval().cpu().state_dict(), name+"_D.pth")
        torch.save(self.G.eval().cpu().state_dict(), name+"_G.pth")
        self.D.train().to(self.dv)
        self.G.train().to(self.dv)

    def load(self, name):
        self.D.load_state_dict(torch.load(name+"_D.pth"))
        self.G.load_state_dict(torch.load(name+"_G.pth"))

    def __repr__(self):
        return "GENERATOR:\n" + self.G.__repr__() + "\nDISCRIMINATOR:\n" + self.D.__repr__()
