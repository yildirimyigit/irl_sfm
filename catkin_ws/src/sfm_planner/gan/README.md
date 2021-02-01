# Generative Adversarial Net
Basic GAN [1] implementation in PyTorch. You can also train WGAN-GP [2] by passing `-w 1` argument. Use GPU if available.

Run:
```bash
python main.py -p DATASET_PATH \
               -z_dim NOISE_DIM \
               -w WASSERSTEIN \
               -bs BATCH_SIZE \
               -lr LEARNING_RATE \
               -d DEPTH \
               -n HIDDEN_UNIT
```
e.g.
```bash
python main.py -p data/input.npy -z_dim 64 -w 1 -bs 128
```

[1] Goodfellow, Ian, et al. "Generative Adversarial Nets." *Advances in Neural Information Processing Systems*. 2014.  
[2] Gulrajani, Ishaan, et al. "Improved Training of Wasserstein GANs." *Advances in Neural Information Processing Systems*. 2017.