#[doc = "Register `pwms` reader"]
pub type R = crate::R<PwmsSpec>;
#[doc = "Register `pwms` writer"]
pub type W = crate::W<PwmsSpec>;
impl core::fmt::Debug for R {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{}", self.bits())
    }
}
impl W {}
#[doc = "Scaled Halfword Counter Register\n\nYou can [`read`](crate::Reg::read) this register and get [`pwms::R`](R). You can [`reset`](crate::Reg::reset), [`write`](crate::Reg::write), [`write_with_zero`](crate::Reg::write_with_zero) this register using [`pwms::W`](W). You can also [`modify`](crate::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct PwmsSpec;
impl crate::RegisterSpec for PwmsSpec {
    type Ux = u32;
}
#[doc = "`read()` method returns [`pwms::R`](R) reader structure"]
impl crate::Readable for PwmsSpec {}
#[doc = "`write(|w| ..)` method takes [`pwms::W`](W) writer structure"]
impl crate::Writable for PwmsSpec {
    type Safety = crate::Unsafe;
}
#[doc = "`reset()` method sets pwms to value 0"]
impl crate::Resettable for PwmsSpec {}
